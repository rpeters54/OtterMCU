
volatile int *iobus_ptr = (volatile int *) 0x00010000;

int fib(int x);

int main(void) {
    for (int i = 0; i < 10; i++) {
        *iobus_ptr = fib(i);
    }
    return 0;
}

int fib(int x) {
    if (x <= 1) {
        return x;
    }
    return fib(x-1) + fib(x-2);
}




