#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>

#include "globals.h"
#include "os.h"


#define THREAD_RUNNING
#define THREAD_READY
#define THREAD_SLEEPING
#define THREAD_WAITING


typedef struct thread_t {
   uint16_t threadID;
   uint16_t stackSize;
   uint16_t stackPtr;
   uint16_t stackBase;
   uint16_t stackEnd;
   uint16_t pc;
   uint16_t sched_count;
} thread_t;


typedef struct system_t {
   thread_t threads[8];
   uint16_t threadCount;
   uint8_t curThreadID;
   uint16_t sysTime;
   uint16_t interrupts;
} system_t;

volatile system_t system;
volatile uint8_t waitlist[8];
volatile uint8_t start;
volatile uint8_t end;

uint8_t get_next_thread() {
   uint8_t ret = 0;
   
   if (system.curThreadID == 0) {
      ret = 1;
   }   
   return ret;
}

//Call this to start the system timer interrupt
   void start_system_timer() {
   //start timer 0 for OS system interrupt
   TIMSK0 |= _BV(OCIE0A);  //interrupt on compare match
   TCCR0A |= _BV(WGM01);   //clear timer on compare match

   //Generate timer interrupt every ~10 milliseconds
   TCCR0B |= _BV(CS02) | _BV(CS00);    //prescalar /1024
   OCR0A = 156;             //generate interrupt every 9.98 milliseconds

   //start timer 1 to generate interrupt every 1 second
   OCR1A = 15625;
   TIMSK1 |= _BV(OCIE1A);  //interrupt on compare
   TCCR1B |= _BV(WGM12) | _BV(CS12) | _BV(CS10); //slowest prescalar /1024
}

__attribute__((naked)) void context_switch(uint16_t* new_tp, uint16_t* old_tp) {
   // push registers onto old stack pointer
   asm volatile ("push r2");
   asm volatile ("push r3");
   asm volatile ("push r4");
   asm volatile ("push r5");
   asm volatile ("push r6");
   asm volatile ("push r7");
   asm volatile ("push r8");
   asm volatile ("push r9");
   asm volatile ("push r10");
   asm volatile ("push r11");
   asm volatile ("push r12");
   asm volatile ("push r13");
   asm volatile ("push r14");
   asm volatile ("push r15");
   asm volatile ("push r16");
   asm volatile ("push r17");
   asm volatile ("push r28");
   asm volatile ("push r29");
   
   
   // The following three blocks will read the data from 0x5E and 0x5D and
   // store those values into r23 and r22, respectively
   asm volatile ("ldi r30, 0x5D");
   asm volatile ("clr r31");
   asm volatile ("ld r10, Z");
   
   asm volatile ("ldi r30, 0x5E");
   asm volatile ("clr r31");
   asm volatile ("ld r11, Z");
   
   asm volatile ("movw r30, r22");
   asm volatile ("st Z+, r10");
   asm volatile ("st Z, r11");
   
   
   // The following three blocks will read the data from r25 and r24 and
   // store those values into memory locations 0x5E and 0x5D, respectively
   asm volatile ("movw r30, r24");  // move
   asm volatile ("ld r10, Z+");
   asm volatile ("ld r11, Z");
   
   
   asm volatile ("ldi r30, 0x5D");
   asm volatile ("clr r31");
   asm volatile ("st Z, r10");
   
   asm volatile ("ldi r30, 0x5E");
   asm volatile ("clr r31");
   asm volatile ("st Z, r11");

   // pop data from the new stack pointer into the registers
   asm volatile ("pop r29");
   asm volatile ("pop r28");
   asm volatile ("pop r17");
   asm volatile ("pop r16");
   asm volatile ("pop r15");
   asm volatile ("pop r14");
   asm volatile ("pop r13");
   asm volatile ("pop r12");
   asm volatile ("pop r11");
   asm volatile ("pop r10");
   asm volatile ("pop r9");
   asm volatile ("pop r8");
   asm volatile ("pop r7");
   asm volatile ("pop r6");
   asm volatile ("pop r5");
   asm volatile ("pop r4");
   asm volatile ("pop r3");
   asm volatile ("pop r2");
   
   
   // do a "ret" instruction at the end, this will fill the PC for us
   asm volatile ("ret");
}


//This interrupt routine is automatically run every 10 milliseconds
ISR(TIMER0_COMPA_vect) {
   //The following statement tells GCC that it can use registers r18-r27, 
   //and r30-31 for this interrupt routine.  These registers (along with
   //r0 and r1) will automatically be pushed and popped by this interrupt routine.
   asm volatile ("" : : : "r18", "r19", "r20", "r21", "r22", "r23", "r24", \
                 "r25", "r26", "r27", "r30", "r31");
   
   volatile uint8_t oldID = system.curThreadID;
   system.interrupts++;
   
   if (system.interrupts % 100 == 0) {
      system.sysTime++;
   }
   
   system.curThreadID = get_next_thread();
   
   context_switch(&(system.threads[system.curThreadID].stackPtr), &(system.threads[oldID].stackPtr));
}

ISR(TIMER1_COMPA_vect) {
   //This interrupt routine is run once a second
   //The 2 interrupt routines will not interrupt each other
   
   
   //Reset the sched_count variables for each thread
   system.interrupts = 0;
   uint8_t i = 0;
   
   for (i = 0; i < system.threadCount; i++) {
      system.threads[i].sched_count = 0;
   }
}


__attribute__((naked)) void thread_start(void) {
   sei();

   asm volatile("movw R30, R16"); // This will move the address of the ThreadFunction into the Z register
   
   asm volatile("ijmp");
}

void os_start() {
   uint16_t dummy;        // Replace dummy with the main thread_t struct
   system.curThreadID = 0;
   
   start_system_timer();  // Starts both Timers
   
   context_switch(&(system.threads[0].stackPtr), &dummy);
}


void create_thread(uint16_t address, void *args, uint16_t stack_size) {
   uint8_t *stackBot = malloc(stack_size + sizeof(struct regs_context_switch) + sizeof(struct regs_interrupt) + 32);
   uint8_t *stackEnd = stackBot + stack_size + sizeof(struct regs_context_switch) + sizeof(struct regs_interrupt) + 32 -1;
   uint8_t *stackPtr = stackEnd - sizeof(struct regs_context_switch);
   
   struct regs_context_switch *regs;
   
   
   regs = (struct regs_context_switch *) (stackPtr);
   
   regs->pcl = (uint16_t)(&thread_start);
   regs->pch = (uint16_t)(&thread_start) >> 8;
   
   regs->r16 = address;       // Low
   regs->r17 = address >> 8;  // High
   
   
   system.threads[system.threadCount].threadID = system.threadCount;
   system.threads[system.threadCount].pc = address;
   system.threads[system.threadCount].sched_count = 0;
   system.threads[system.threadCount].stackSize = stack_size + sizeof(struct regs_context_switch) + sizeof(struct regs_interrupt) + 32;
   
   system.threads[system.threadCount].stackBase = (uint16_t) stackBot;
   system.threads[system.threadCount].stackPtr = (uint16_t) stackPtr;
   system.threads[system.threadCount].stackEnd = (uint16_t) (stackBot + system.threads[system.threadCount].stackSize - 1);
   
   system.threadCount++;
}


void led_on() {
   asm volatile ("ldi r30, 0x24");
   asm volatile ("clr r31");
   asm volatile ("ld r21, Z");
   asm volatile ("ldi r22, 0x20");
   asm volatile ("or r21, r22"); 
   asm volatile ("st Z, r21");
   
   asm volatile ("ldi r30, 0x25");
   asm volatile ("clr r31");
   asm volatile ("ld r23, Z"); 
   asm volatile ("or r23, r22");
   asm volatile ("st Z, r23");
}
   
void led_off() {
   asm volatile ("ldi r30, 0x24");
   asm volatile ("clr r31");
   asm volatile ("ld r21, Z");
   asm volatile ("ldi r22, 0x20");
   asm volatile ("or r21, r22");
   asm volatile ("st Z, r21");
   asm volatile ("com r22");
   
   asm volatile ("ldi r30, 0x25");
   asm volatile ("clr r31");
   asm volatile ("ld r23, Z"); 
   asm volatile ("ldi r23, 0x25");  
   asm volatile ("and r23, r22");
   asm volatile ("st Z, r23");
}

void blink() {
   while(1) {
      // include code from lab1 part 3
      led_on();
      _delay_ms(250);
      led_off();  
      _delay_ms(250);
   }
}


void display_stats() {
   int i;
   
   system.threads[system.curThreadID].sched_count++;
   
   while(1) {
      // Print stats
      clear_screen();
      set_cursor(1, 1);
      
      print_string("System Time (s): ");
      print_int(system.sysTime);
      
      print_string("\n\rInterrupts/sec: ");
      print_int(system.interrupts);
      
      print_string("\n\rThread Count: ");
      print_int(system.threadCount);
      
      for (i = 0; i < system.threadCount; i++) {
         print_string("\n\r\nThread ID: ");
         print_int(i);
         
         print_string("\n\rThread PC: ");
         print_hex(system.threads[i].pc);
         
         print_string("\n\rStack Usage: ");
         print_int(system.threads[i].stackEnd - system.threads[i].stackPtr);
         
         print_string("\n\rTotal Stack Space: ");
         print_int(system.threads[i].stackSize);
         
         print_string("\n\rCurrent Stack Top: ");
         print_int(system.threads[i].stackPtr);
         
         print_string("\n\rStack Base: ");
         print_int(system.threads[i].stackBase);
         
         print_string("\n\rStack End: ");
         print_int(system.threads[i].stackEnd);
      }
      
      _delay_ms(400);
   }
}


void display_bounded_buffer();


void producer();


void consumer();


int main() {
   
   system.threadCount = 0;
   system.sysTime = 0;
   system.interrupts = 0;
   
   // Initialize fields in the main thread
   
   serial_init();
   
   create_thread((uint16_t) blink, NULL, 32);
   create_thread((uint16_t) display_stats, NULL, 32);
   create_thread((uint16_t) display_bounded_buffer, NULL, 32);
   create_thread((uint16_t) producer, NULL, 32);
   create_thread((uint16_t) consumer, NULL, 32);
   
   os_start();
   
   while(1) {
   }
}
