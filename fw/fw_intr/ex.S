
fw_intr.elf:     file format elf32-littleriscv


Disassembly of section .text.init:

80001000 <rvtest_entry_point>:
80001000:	00003117          	auipc	sp,0x3
80001004:	00010113          	mv	sp,sp
80001008:	001002b7          	lui	t0,0x100
8000100c:	00510133          	add	sp,sp,t0
80001010:	00003297          	auipc	t0,0x3
80001014:	ff028293          	addi	t0,t0,-16 # 80004000 <interrupt_fired_flag>
80001018:	00003317          	auipc	t1,0x3
8000101c:	fe830313          	addi	t1,t1,-24 # 80004000 <interrupt_fired_flag>

80001020 <clear_bss_loop>:
80001020:	00628863          	beq	t0,t1,80001030 <clear_bss_done>
80001024:	0002a023          	sw	zero,0(t0)
80001028:	00428293          	addi	t0,t0,4
8000102c:	ff5ff06f          	j	80001020 <clear_bss_loop>

80001030 <clear_bss_done>:
80001030:	01c010ef          	jal	8000204c <main>

80001034 <end_loop>:
80001034:	0000006f          	j	80001034 <end_loop>

Disassembly of section .text:

80002000 <interrupt_handler>:
80002000:	ff810113          	addi	sp,sp,-8 # 80003ff8 <INTRPT_READY+0xff4>
80002004:	00512023          	sw	t0,0(sp)
80002008:	00612223          	sw	t1,4(sp)
8000200c:	00002297          	auipc	t0,0x2
80002010:	ff428293          	addi	t0,t0,-12 # 80004000 <interrupt_fired_flag>
80002014:	00100313          	li	t1,1
80002018:	0062a023          	sw	t1,0(t0)
8000201c:	00412303          	lw	t1,4(sp)
80002020:	00012283          	lw	t0,0(sp)
80002024:	00810113          	addi	sp,sp,8
80002028:	30200073          	mret

8000202c <setup_interrupts>:
8000202c:	00000297          	auipc	t0,0x0
80002030:	fd428293          	addi	t0,t0,-44 # 80002000 <interrupt_handler>
80002034:	30529073          	csrw	mtvec,t0
80002038:	30401073          	csrw	mie,zero
8000203c:	30452073          	csrs	mie,a0
80002040:	00800293          	li	t0,8
80002044:	3002a073          	csrs	mstatus,t0
80002048:	00008067          	ret

8000204c <main>:
8000204c:	fe010113          	addi	sp,sp,-32
80002050:	00112e23          	sw	ra,28(sp)
80002054:	00812c23          	sw	s0,24(sp)
80002058:	02010413          	addi	s0,sp,32
8000205c:	00100793          	li	a5,1
80002060:	fef42623          	sw	a5,-20(s0)
80002064:	0740006f          	j	800020d8 <main+0x8c>
80002068:	ffff17b7          	lui	a5,0xffff1
8000206c:	88878793          	addi	a5,a5,-1912 # ffff0888 <interrupt_fired_flag+0x7ffec888>
80002070:	00078713          	mv	a4,a5
80002074:	fec42783          	lw	a5,-20(s0)
80002078:	00f777b3          	and	a5,a4,a5
8000207c:	04078663          	beqz	a5,800020c8 <main+0x7c>
80002080:	fec42783          	lw	a5,-20(s0)
80002084:	00078513          	mv	a0,a5
80002088:	fa5ff0ef          	jal	8000202c <setup_interrupts>
8000208c:	800037b7          	lui	a5,0x80003
80002090:	0047a783          	lw	a5,4(a5) # 80003004 <INTRPT_READY>
80002094:	fec42703          	lw	a4,-20(s0)
80002098:	00e7a023          	sw	a4,0(a5)
8000209c:	00000013          	nop
800020a0:	800047b7          	lui	a5,0x80004
800020a4:	0007a783          	lw	a5,0(a5) # 80004000 <interrupt_fired_flag>
800020a8:	fe078ce3          	beqz	a5,800020a0 <main+0x54>
800020ac:	800037b7          	lui	a5,0x80003
800020b0:	0007a783          	lw	a5,0(a5) # 80003000 <MMIO_FINISH_ADDR>
800020b4:	fec42703          	lw	a4,-20(s0)
800020b8:	00e7a023          	sw	a4,0(a5)
800020bc:	800047b7          	lui	a5,0x80004
800020c0:	0007a023          	sw	zero,0(a5) # 80004000 <interrupt_fired_flag>
800020c4:	0080006f          	j	800020cc <main+0x80>
800020c8:	00000013          	nop
800020cc:	fec42783          	lw	a5,-20(s0)
800020d0:	00179793          	slli	a5,a5,0x1
800020d4:	fef42623          	sw	a5,-20(s0)
800020d8:	fec42783          	lw	a5,-20(s0)
800020dc:	f80796e3          	bnez	a5,80002068 <main+0x1c>
800020e0:	0000006f          	j	800020e0 <main+0x94>
