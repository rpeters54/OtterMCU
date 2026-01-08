
fw_csr.elf:     file format elf32-littleriscv


Disassembly of section .text.init:

80001000 <rvtest_entry_point>:
80001000:	00002117          	auipc	sp,0x2
80001004:	00010113          	mv	sp,sp
80001008:	001002b7          	lui	t0,0x100
8000100c:	00510133          	add	sp,sp,t0
80001010:	00002297          	auipc	t0,0x2
80001014:	ff028293          	addi	t0,t0,-16 # 80003000 <__bss_end>
80001018:	00002317          	auipc	t1,0x2
8000101c:	fe830313          	addi	t1,t1,-24 # 80003000 <__bss_end>

80001020 <clear_bss_loop>:
80001020:	00628863          	beq	t0,t1,80001030 <clear_bss_done>
80001024:	0002a023          	sw	zero,0(t0)
80001028:	00428293          	addi	t0,t0,4
8000102c:	ff5ff06f          	j	80001020 <clear_bss_loop>

80001030 <clear_bss_done>:
80001030:	144010ef          	jal	80002174 <main>

80001034 <end_loop>:
80001034:	0000006f          	j	80001034 <end_loop>

Disassembly of section .text:

80002000 <csr_test>:
80002000:	00000297          	auipc	t0,0x0
80002004:	16028293          	addi	t0,t0,352 # 80002160 <trap_handler>
80002008:	30529073          	csrw	mtvec,t0
8000200c:	00800313          	li	t1,8
80002010:	30033073          	csrc	mstatus,t1
80002014:	00010437          	lui	s0,0x10
80002018:	34001073          	csrw	mscratch,zero
8000201c:	a5a5a337          	lui	t1,0xa5a5a
80002020:	5a530313          	addi	t1,t1,1445 # a5a5a5a5 <__bss_end+0x25a575a5>
80002024:	340312f3          	csrrw	t0,mscratch,t1
80002028:	00542023          	sw	t0,0(s0) # 10000 <OUT_BASE>
8000202c:	00440413          	addi	s0,s0,4
80002030:	340023f3          	csrr	t2,mscratch
80002034:	00742023          	sw	t2,0(s0)
80002038:	00440413          	addi	s0,s0,4
8000203c:	00001eb7          	lui	t4,0x1
80002040:	f0fe8e93          	addi	t4,t4,-241 # f0f <OUT_BASE-0xf0f1>
80002044:	340ea2f3          	csrrs	t0,mscratch,t4
80002048:	00542023          	sw	t0,0(s0)
8000204c:	00440413          	addi	s0,s0,4
80002050:	340023f3          	csrr	t2,mscratch
80002054:	00742023          	sw	t2,0(s0)
80002058:	00440413          	addi	s0,s0,4
8000205c:	340022f3          	csrr	t0,mscratch
80002060:	00542023          	sw	t0,0(s0)
80002064:	00440413          	addi	s0,s0,4
80002068:	340024f3          	csrr	s1,mscratch
8000206c:	00942023          	sw	s1,0(s0)
80002070:	00440413          	addi	s0,s0,4
80002074:	0f000e93          	li	t4,240
80002078:	340eb2f3          	csrrc	t0,mscratch,t4
8000207c:	00542023          	sw	t0,0(s0)
80002080:	00440413          	addi	s0,s0,4
80002084:	340023f3          	csrr	t2,mscratch
80002088:	00742023          	sw	t2,0(s0)
8000208c:	00440413          	addi	s0,s0,4
80002090:	340032f3          	csrrc	t0,mscratch,zero
80002094:	00542023          	sw	t0,0(s0)
80002098:	00440413          	addi	s0,s0,4
8000209c:	340024f3          	csrr	s1,mscratch
800020a0:	00942023          	sw	s1,0(s0)
800020a4:	00440413          	addi	s0,s0,4
800020a8:	340fd2f3          	csrrwi	t0,mscratch,31
800020ac:	00542023          	sw	t0,0(s0)
800020b0:	00440413          	addi	s0,s0,4
800020b4:	340023f3          	csrr	t2,mscratch
800020b8:	00742023          	sw	t2,0(s0)
800020bc:	00440413          	addi	s0,s0,4
800020c0:	34001073          	csrw	mscratch,zero
800020c4:	340ae2f3          	csrrsi	t0,mscratch,21
800020c8:	00542023          	sw	t0,0(s0)
800020cc:	00440413          	addi	s0,s0,4
800020d0:	340023f3          	csrr	t2,mscratch
800020d4:	00742023          	sw	t2,0(s0)
800020d8:	00440413          	addi	s0,s0,4
800020dc:	340062f3          	csrrsi	t0,mscratch,0
800020e0:	00542023          	sw	t0,0(s0)
800020e4:	00440413          	addi	s0,s0,4
800020e8:	340024f3          	csrr	s1,mscratch
800020ec:	00942023          	sw	s1,0(s0)
800020f0:	00440413          	addi	s0,s0,4
800020f4:	0ff00293          	li	t0,255
800020f8:	34029073          	csrw	mscratch,t0
800020fc:	3407f2f3          	csrrci	t0,mscratch,15
80002100:	00542023          	sw	t0,0(s0)
80002104:	00440413          	addi	s0,s0,4
80002108:	340023f3          	csrr	t2,mscratch
8000210c:	00742023          	sw	t2,0(s0)
80002110:	00440413          	addi	s0,s0,4
80002114:	340072f3          	csrrci	t0,mscratch,0
80002118:	00542023          	sw	t0,0(s0)
8000211c:	00440413          	addi	s0,s0,4
80002120:	340024f3          	csrr	s1,mscratch
80002124:	00942023          	sw	s1,0(s0)
80002128:	00440413          	addi	s0,s0,4
8000212c:	00100073          	ebreak
80002130:	342022f3          	csrr	t0,mcause
80002134:	00542023          	sw	t0,0(s0)
80002138:	00440413          	addi	s0,s0,4
8000213c:	00000073          	ecall
80002140:	342022f3          	csrr	t0,mcause
80002144:	00542023          	sw	t0,0(s0)
80002148:	00440413          	addi	s0,s0,4
8000214c:	10500073          	wfi
80002150:	342022f3          	csrr	t0,mcause
80002154:	00542023          	sw	t0,0(s0)
80002158:	00440413          	addi	s0,s0,4
8000215c:	0000006f          	j	8000215c <csr_test+0x15c>

80002160 <trap_handler>:
80002160:	341022f3          	csrr	t0,mepc
80002164:	00428293          	addi	t0,t0,4
80002168:	34129073          	csrw	mepc,t0
8000216c:	30200073          	mret
80002170:	00000013          	nop

80002174 <main>:
80002174:	ff010113          	addi	sp,sp,-16 # 80002ff0 <main+0xe7c>
80002178:	00112623          	sw	ra,12(sp)
8000217c:	00812423          	sw	s0,8(sp)
80002180:	01010413          	addi	s0,sp,16
80002184:	e7dff0ef          	jal	80002000 <csr_test>
80002188:	00000793          	li	a5,0
8000218c:	00078513          	mv	a0,a5
80002190:	00c12083          	lw	ra,12(sp)
80002194:	00812403          	lw	s0,8(sp)
80002198:	01010113          	addi	sp,sp,16
8000219c:	00008067          	ret
