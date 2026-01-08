
volatile int interrupt_fired_flag = 0;
// A memory-mapped I/O address to signal test completion.
volatile int *MMIO_FINISH_ADDR = (volatile int *)0x00010000;
volatile int *INTRPT_READY     = (volatile int *)0x00020000;
const int MIE_MASK = 0xFFFF0888;

// Declaration for our assembly function that sets up the CSRs.
void setup_interrupts(int mie);

int main() {
    for(unsigned int mie = 1; mie > 0; mie <<= 1) {
        if (!(mie & MIE_MASK)) {
            continue;
        }

        setup_interrupts(mie);

        *INTRPT_READY = mie;

        while (interrupt_fired_flag == 0);

        *MMIO_FINISH_ADDR = mie;

        interrupt_fired_flag = 0;
    }

    for(;;);
}
