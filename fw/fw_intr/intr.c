volatile int interrupt_fired_flag = 0;

// A memory-mapped I/O address to signal test completion.
#define MMIO_FINISH_ADDR (*(volatile int*)0x10000)

// Declaration for our assembly function that sets up the CSRs.
void setup_interrupts();

int main() {
    setup_interrupts();

    // Loop indefinitely, waiting for an interrupt to change the flag.
    while (interrupt_fired_flag == 0);

    // MMIO write that can easily tracked by the testbench
    MMIO_FINISH_ADDR = 1;

    while(1);

    return 0;
}
