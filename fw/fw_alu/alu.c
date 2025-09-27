
volatile int *iobus_ptr = (volatile int *) 0xFFFF0000;

void test_r_types();
void test_i_types();

int main(void) {
    test_r_types();
    test_i_types();

    return 0;
}

/**
 * @brief Tests R-type instructions using inline assembly.
 */
void test_r_types() {
    int result;
    int a = 20;
    int b = -10;
    unsigned int ua = 20;
    unsigned int ub = 30;

    __asm__ volatile ("add %0, %1, %2"  : "=r"(result) : "r"(a), "r"(b));
    *iobus_ptr = result;

    __asm__ volatile ("sub %0, %1, %2"  : "=r"(result) : "r"(a), "r"(b));
    *iobus_ptr = result;

    __asm__ volatile ("sll %0, %1, %2"  : "=r"(result) : "r"(a), "r"(b));
    *iobus_ptr = result;

    __asm__ volatile ("slt %0, %1, %2"  : "=r"(result) : "r"(a), "r"(b));
    *iobus_ptr = result;
    __asm__ volatile ("slt %0, %1, %2"  : "=r"(result) : "r"(b), "r"(a)); // Flipped for true case
    *iobus_ptr = result;

    __asm__ volatile ("sltu %0, %1, %2" : "=r"(result) : "r"(ua), "r"(ub));
    *iobus_ptr = result;
    __asm__ volatile ("sltu %0, %1, %2" : "=r"(result) : "r"(ub), "r"(ua)); // Flipped for false case
    *iobus_ptr = result;

    __asm__ volatile ("xor %0, %1, %2"  : "=r"(result) : "r"(a), "r"(b));
    *iobus_ptr = result;

    __asm__ volatile ("srl %0, %1, %2"  : "=r"(result) : "r"(ua), "r"(b));
    *iobus_ptr = result;

    __asm__ volatile ("sra %0, %1, %2"  : "=r"(result) : "r"(a), "r"(b));
    *iobus_ptr = result;
    __asm__ volatile ("sra %0, %1, %2"  : "=r"(result) : "r"(b), "r"(a)); // Test with negative number
    *iobus_ptr = result;

    __asm__ volatile ("or %0, %1, %2"   : "=r"(result) : "r"(a), "r"(b));
    *iobus_ptr = result;

    __asm__ volatile ("and %0, %1, %2"  : "=r"(result) : "r"(a), "r"(b));
    *iobus_ptr = result;
}


/**
 * @brief Tests I-type instructions using inline assembly.
 */
void test_i_types() {
    int result;
    int a = 50;
    int neg_a = -50;
    unsigned int ua = 50;

    __asm__ volatile ("addi %0, %1, 15"   : "=r"(result) : "r"(a));
    *iobus_ptr = result;
    __asm__ volatile ("addi %0, %1, -15"  : "=r"(result) : "r"(a));
    *iobus_ptr = result;

    __asm__ volatile ("slti %0, %1, 15"   : "=r"(result) : "r"(a));
    *iobus_ptr = result;
    __asm__ volatile ("slti %0, %1, 100"  : "=r"(result) : "r"(a));
    *iobus_ptr = result;

    __asm__ volatile ("sltiu %0, %1, 15"  : "=r"(result) : "r"(ua));
    *iobus_ptr = result;
    __asm__ volatile ("sltiu %0, %1, 100" : "=r"(result) : "r"(ua));
    *iobus_ptr = result;

    __asm__ volatile ("xori %0, %1, 15"   : "=r"(result) : "r"(a));
    *iobus_ptr = result;

    __asm__ volatile ("ori %0, %1, 15"    : "=r"(result) : "r"(a));
    *iobus_ptr = result;

    __asm__ volatile ("andi %0, %1, 15"   : "=r"(result) : "r"(a));
    *iobus_ptr = result;

    __asm__ volatile ("slli %0, %1, 3"    : "=r"(result) : "r"(a));
    *iobus_ptr = result;

    __asm__ volatile ("srli %0, %1, 3"    : "=r"(result) : "r"(ua));
    *iobus_ptr = result;

    __asm__ volatile ("srai %0, %1, 3"    : "=r"(result) : "r"(a));
    *iobus_ptr = result;
    __asm__ volatile ("srai %0, %1, 3"    : "=r"(result) : "r"(neg_a));
    *iobus_ptr = result;
}
