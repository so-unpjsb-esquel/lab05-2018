// Simple PIO-based (non-DMA) IDE driver code.
// Con comentarios extras (ver tmb archivo IDE.txt)

#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "fs.h"
#include "buf.h"

#define SECTOR_SIZE   512

// Indicates the drive is preparing to send/receive data (wait for it 
// to clear). In case of 'hang' (it never clears), do a software reset.
#define IDE_BSY       

// Bit is clear when drive is spun down, or after an error. Set otherwise. 
#define IDE_DRDY      0x40

// Drive Fault Error (does not set ERR).
#define IDE_DF        0x20

// Indicates an error occurred. Send a new command to clear it (or 
// nuke it with a Software Reset).
#define IDE_ERR       0x01

#define IDE_CMD_READ  0x20
#define IDE_CMD_WRITE 0x30
#define IDE_CMD_RDMUL 0xc4
#define IDE_CMD_WRMUL 0xc5

// idequeue points to the buf now being read/written to the disk.
// idequeue->qnext points to the next buf to be processed.
// You must hold idelock while manipulating queue.

static struct spinlock idelock;
static struct buf *idequeue;

static int havedisk1;
static void idestart(struct buf*);

// Wait for IDE disk to become ready.
static int
idewait(int checkerr)
{
  int r;

  /* 0x1f7 es el puerto de comandos de la controladora IDE, que
   * permite enviar comandos o leer el estado actual del disco.
   * Con inb(0x1f7) se lee el estado del disco (status register).
   * 
   * Luego, se hace un and (&) entre el valor del status register,
   * guardado en r, y la bandera IDE_BSY | IDE_DRDY. Mientras el
   * valor sea distinto de IDE_DRDY, se mantiene en una espera
   * activa.
   */
  while(((r = inb(0x1f7)) & (IDE_BSY|IDE_DRDY)) != IDE_DRDY)
    ;

   /* (r & (IDE_DF|IDE_ERR)) es un valor distinto de cero si ocurrio
   * algún error en la controladora. Si checkerr > 0, entonces se 
   * retorna -1 en caso de error.
   */
  if(checkerr && (r & (IDE_DF|IDE_ERR)) != 0)
    return -1;

  return 0;
}

void
ideinit(void)
{
  int i;

  // initialize a mutual exclusion spinlock called "ide"
  initlock(&idelock, "ide");
  ioapicenable(IRQ_IDE, ncpu - 1);
  
  // wait for the hard-disk drive to be ready
  idewait(0);

  // Check if disk 1 is present
  /*
   * 0x1f6 es el registro selectord de disco / cabezal
   * El bit 4 en el registro indica el disco sobre el 
   * que operar. Si es 0, es el disco rigido primario, 
   * y si es 1, selecciona el disco secundario. Con la
   * expresión (1<<4), se pone en 1 el bit 4.
   */  
  outb(0x1f6, 0xe0 | (1<<4));
  
  /*
   * Verifica repetidamente el valor del registro de 
   * estado (0x1f7), hasta que sea distinto de cero. Si 
   * es el caso, es que hay un disco secundario presente.
   */
  for(i=0; i<1000; i++){
    if(inb(0x1f7) != 0){
      havedisk1 = 1;
      break;
    }
  }

  // Switch back to disk 0.
  outb(0x1f6, 0xe0 | (0<<4));
}

// Start the request for b.  Caller must hold idelock.
static void
idestart(struct buf *b)
{
  if(b == 0)
    panic("idestart");  // se invoco idestart() con un puntero inválido
  
  if(b->blockno >= FSSIZE)
    panic("incorrect blockno");
  
  int sector_per_block =  BSIZE/SECTOR_SIZE;
  int sector = b->blockno * sector_per_block;
  int read_cmd = (sector_per_block == 1) ? IDE_CMD_READ :  IDE_CMD_RDMUL;
  int write_cmd = (sector_per_block == 1) ? IDE_CMD_WRITE : IDE_CMD_WRMUL;

  if (sector_per_block > 7) panic("idestart");

  idewait(0);  // espera a que el disco este listo
  
  /*
   * Generate interrupt. If 1, stop the current device from sending interrupts.
   */
  outb(0x3f6, 0);  // generate interrupt
  
  /*
   * 0x1f2: SECTOR COUNT REGISTER
   * Defines the number of sectors of data to be transferred across 
   * the host bus, for the subsequent command.   
   */
  outb(0x1f2, sector_per_block);  // number of sectors
  
  /*
   * 0x1f3: SECTOR NUMBER REGISTER
   * Contains the ID number of the first sector to be accessed by the
   * subsequent command. The sector can be from one to the maximum 
   * number of sectors per track.
   */
  outb(0x1f3, sector & 0xff);
  
  // El disco se accede en modo LBA (Logical Block Addressing), con lo cual 
  // se pasa el número de sector, y no Cilinder-Head-Sector
  
  /* 0x1f4: CYLINDER LOW REGISTER
   * Contains the eight low order bits of the starting cylinder 
   * address for any disk access.
   */
  outb(0x1f4, (sector >> 8) & 0xff);
  
  /* 0x1f5: CYLINDER HIGH REGISTER
   * Contains the eight high order bits of the starting cylinder 
   * address for any disk access.
   */ 
  outb(0x1f5, (sector >> 16) & 0xff);
  
  /* 0x1f6: DRIVE/HEAD REGISTER
   * Contains the drive ID number and its head number for any disk 
   * access.
   * e0: ‭11100000‬ (estos bits ya estan seteados por defecto)
   * (b->dev & 1) << 4 (el disco del cual leer/escribir)
   * (b->sector >> 24) & 0x0f (el numero de sector)
   */
  outb(0x1f6, 0xe0 | ((b->dev&1)<<4) | ((sector>>24)&0x0f));
  
  // buffer needs to be written to disk?
  if(b->flags & B_DIRTY){
    // 0x1f7: COMMAND REGISTER (write)
    // Write IDE_CMD_WRITE command in 0x1f7 port.
    outb(0x1f7, write_cmd);
    
    // 0x1f70 DATA PORT REGISTER
    // All data transferred between the device data buffer and the host
    // passes through this register.
    outsl(0x1f0, b->data, BSIZE/4);
  } else {
    // Write IDE_CMD_READ command in 0x1f7 port.
    outb(0x1f7, read_cmd);
  }
}

// Interrupt handler.
void
ideintr(void)
{
  struct buf *b;

  // First queued buffer is the active request.
  // acquire the mutual exclusion spinlock "idelock"
  acquire(&idelock);

  // Check for a spurious IDE interrupt
  if((b = idequeue) == 0){
    release(&idelock);
    return;
  }
  
  // b->qnext is now the new head of the list
  idequeue = b->qnext;

  // Read data if needed.
  // 0x1f0 DATA PORT REGISTER
  // All data transferred between the device data buffer and the host
  // passes through this register.
  if(!(b->flags & B_DIRTY) && idewait(1) >= 0)
    insl(0x1f0, b->data, BSIZE/4);

  // Wake process waiting for this buf.
  b->flags |= B_VALID;  // set that this buffer has been read from the disk
  b->flags &= ~B_DIRTY; // clear dirty bit in flags (we have fresh bytes from the hard-disk!)
  wakeup(b);            // wake-up the process waiting for buffer b (see proc.c)

  // Start disk on next buf in queue.
  if(idequeue != 0)
    idestart(idequeue); // next buffer

  release(&idelock);
}

//PAGEBREAK!
// Sync buf with disk.
// If B_DIRTY is set, write buf to disk, clear B_DIRTY, set B_VALID.
// Else if B_VALID is not set, read buf from disk, set B_VALID.
void
iderw(struct buf *b)
{
  struct buf **pp;

  if(!holdingsleep(&b->lock))
    panic("iderw: buf not locked");
  if((b->flags & (B_VALID|B_DIRTY)) == B_VALID)
    panic("iderw: nothing to do");
  if(b->dev != 0 && !havedisk1)
    panic("iderw: ide disk 1 not present");

  acquire(&idelock);  //DOC:acquire-lock

  // Append b to idequeue.
  b->qnext = 0;
  for(pp=&idequeue; *pp; pp=&(*pp)->qnext)  //DOC:insert-queue
    ;
  *pp = b;

  // Start disk if necessary.
  if(idequeue == b)
    idestart(b); // start the request -- when finished, a interrupt will be generated, and processed by ideintr()

  // Wait for request to finish.
  while((b->flags & (B_VALID|B_DIRTY)) != B_VALID){
    // Releases idelock and put the process to sleep. Then reacquires lock 
    // when reawakened. See proc.c for details.
    sleep(b, &idelock);
  }


  release(&idelock);  // release idelock, so other process can access the drive
}
