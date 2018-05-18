 #
# if you want the ram-disk device, define this to be the
# size in blocks.
# 如果要使用 RAM 就定义块的大小(注释掉了)，这是一个编译时参数，如果定义了在下面会用到。
RAMDISK = #-DRAMDISK=512

AS86	=as86 -0 -a  #8006汇编的编译器和连接器。后面参数分别是：
LD86	=ld86 -0 	 # -0 生成8086目标程序， -a 生成与gas 和 gld部分兼容的代码。

AS	=gas  #GNU 汇编器和连接器。
LD	=gld
LDFLAGS	=-s -x -M  # gld 参数，-s 输出文件中省略所有符号信息， -x 删除所有局部符号， -m 在标准输出设备上打印连接映像
                   #连接映像是指由连接程序程序产生的一种内存地址映像，列出了程序段装到内存中的位置信息。具体指：
                   #1.目标文件及符号信息映射到内存中的位置
                   #2.公告符号如何放置
                   #3.连接中包含的所有文件及其引用的符号

# gcc GNU编译器，引用定义的符号时，需要在前面加上$符号，并用括号把定义的标识符括起来。 
CC	=gcc $(RAMDISK)

# -Wall 打印所有警告信息， -O 对代码进行优化   "-f标志" 指定与机器无关的编译标志
# 1. -fstrength-reduce   用于优化循环语句
# 2. -fomit-frame-pointer  指明对无需帧指针(Frame pointer)的函数不要把帧指针保留在寄存器中，可以避免对帧指针的操作和维护。
# 3. -fcombine-regs 指明编译器在组合编译阶段把复制一个寄存器到另一个寄存器的指令组合在一起。
# 4. -mstring-insns linus 在学习gcc时为gcc增加的选项，用于 gcc-1.40 在复制结构等操作时使用386cpu的字符串指令，可以去掉。

CFLAGS	=-Wall -O -fstrength-reduce -fomit-frame-pointer \
-fcombine-regs -mstring-insns

# -nostdinc -Iinclude 不要搜索标准头文件目录中的文件，即不用 /usr/include/目录下的头文件，
# 而是使用 "-I" 选项指定的目录或者是在当前目录里搜索头文件。
CPP	=cpp -nostdinc -Iinclude

#
# ROOT_DEV specifies the default root-device when making the image.
# This can be either FLOPPY, /dev/xxxx or empty, in which case the
# default of /dev/hd6 is used by 'build'.
#  
# ROOT_DEV 指定在创建内核映像文件时所使用的默认根文件系统所在的设备，可以是软盘、
# /dev/xxx 或者空， 空着时使用默认值 /dev/hd6
#
ROOT_DEV=/dev/hd6


# kernel目录， mm目录，和fs目录所产生的目标代码文件，为了方便引用，用ARCHIVES(归档文件)标识符标书
ARCHIVES=kernel/kernel.o mm/mm.o fs/fs.o

# 块和字符设备库文件。 '.a'表示该文件是个归档文件，即包含许多可执行二进制代码
#子程序集合的库文件，通常由 GNU的 ar程序生成， ar 是GNU的二进制文件处理程序，用于创建、修改以及从归档文件中抽取文件。
DRIVERS =kernel/blk_drv/blk_drv.a kernel/chr_drv/chr_drv.a

MATH	=kernel/math/math.a  # 数学运算库文件
LIBS	=lib/lib.a  # 由 lib/目录中文件所编译生成的库文件。


#1. make 老式的隐式后缀规则，指示make利用下面的命令将所欲的'.c'文件编译生成'.s'汇编程序。
#2. 使用 include/ 目录下的头文件， 
#3. -S 表示只进行编译，产生与各个C文件对应的汇编文件。默认情况下编译产生的文件名是源文件去掉'.c'后再加上'.s'后缀。
#4. -o 后面是输出的文件的格式,其中'$*.s'(或'$@')是自动目标变量。
#5. '$<' 代表第一个先决条件，这里即符合条件的'*.c' 文件

#下面有三个规则：
#1.若目标文件是'.s'文件,源文件是'.c'文件，则使用第一个规则。
#2.若目标文件是'.o'文件,源文件是'.s'文件，则使用第二个规则。
#3.若目标文件是'.o'文件,源文件是'.c'文件，则使用第三个规则。

.c.s:
	$(CC) $(CFLAGS) \
	-nostdinc -Iinclude -S -o $*.s $<
.s.o:
	$(AS) -c -o $*.o $<
.c.o:
	$(CC) $(CFLAGS) \
	-nostdinc -Iinclude -c -o $*.o $<

# all 表示makefile所知的最顶层目标，这里是Image文件，这里是引导启动盘映像文件bootimage。
all:	Image

# 冒号后面的文件是生成 Image 文件依赖的4个文件，下一行是生成Image的执行命令
Image: boot/bootsect boot/setup tools/system tools/build
	tools/build boot/bootsect boot/setup tools/system $(ROOT_DEV) > Image #生成Image文件的命令
	sync  #使用同步命令迫使缓冲块数据立即写盘并更新超级块。

# disk 目标文件由 Image产生。dd命令：复制一个文件，根据选项进行转换的格式化。
# bs= 表示第一次读/写的字节数。 if= 表示输入的文件。 of= 表示输出到文件。
disk: Image
	dd bs=8192 if=Image of=/dev/PS0  #使用dd命令把Image文件写入/dev/PS0(第一个软盘驱动器)

# 编译生成 tools/build 文件
tools/build: tools/build.c
	$(CC) $(CFLAGS) \
	-o tools/build tools/build.c

# 利用上面给出的 .s.o 规则生成 head.o文件
boot/head.o: boot/head.s

# 编译生成 tools/system文件
tools/system:	boot/head.o init/main.o \
		$(ARCHIVES) $(DRIVERS) $(MATH) $(LIBS)
	$(LD) $(LDFLAGS) boot/head.o init/main.o \
	$(ARCHIVES) \
	$(DRIVERS) \
	$(MATH) \
	$(LIBS) \
	-o tools/system > System.map

# 生成数学协处理文件，math.a，进入 kernel/math 目录，运行该目录下的makefile。下面的几条命令类似这样。
kernel/math/math.a:
	(cd kernel/math; make)

#生成块设备库文件 blk_drv.a，其中含有可重定位目标文件。
kernel/blk_drv/blk_drv.a:
	(cd kernel/blk_drv; make)

#生成字符设备函数文件 chr_drv.a
kernel/chr_drv/chr_drv.a:
	(cd kernel/chr_drv; make)

#生成内核目标模块 kernel.o
kernel/kernel.o:
	(cd kernel; make)

#生成内存管理模块 mm.o
mm/mm.o:
	(cd mm; make)

#生成文件系统目标模块fs.o
fs/fs.o:
	(cd fs; make)

#生成库函数liba.a
lib/lib.a:
	(cd lib; make)

boot/setup: boot/setup.s                     #这里三行使用8086汇编器和连接器对
	$(AS86) -o boot/setup.o boot/setup.s     #setup.s 文件进行编译生成setup文件
	$(LD86) -s -o boot/setup boot/setup.o    # -s 表示去除目标文件中的符号信息

# 同上，生成 bootsect 磁盘引导块
boot/bootsect:	boot/bootsect.s
	$(AS86) -o boot/bootsect.o boot/bootsect.s
	$(LD86) -s -o boot/bootsect boot/bootsect.o

# 在bootsect.s文本程序开始处添加一行有关system模块文件长度信息，在把system模块
# 加载到内存期间用于指明系统模块的长度。方法是是利用命令获取 system模块的大小，并保存
# 在tmp.s 文件中。cut命令用于剪切字符串，tr用于去掉行尾的回车符。(实际长度+15)/16 用于
# 获得"节"表示的长度信息， 1节 = 16 字节。 这是旧版本(0.01-0.10)在使用，新版本已经不用
# 新版本直接在文件中指明了 system的大小。
tmp.s:	boot/bootsect.s tools/system
	(echo -n "SYSSIZE = (";ls -l tools/system | grep system \
		| cut -c25-31 | tr '\012' ' '; echo "+ 15 ) / 16") > tmp.s
	cat boot/bootsect.s >> tmp.s

# 执行 make clean 时执以下命令，删除编译链接生成的文件
# rm 是文件删除命令， -f 表示忽略不存在的文件，并且不显示删除信息。
clean:
	rm -f Image System.map tmp_make core boot/bootsect boot/setup
	rm -f init/*.o tools/system tools/build boot/*.o
	(cd mm;make clean)  #进入 mm/目录，执行该目录下的makefile文件中的clean规则，下面类似。
	(cd fs;make clean)
	(cd kernel;make clean)
	(cd lib;make clean)

#该规则先执行上面的clean规则，然后对 linux/目录进行压缩，生成 backup.Z 压缩文件。
# cd.. 退到Linux的上一级目录，tar cf - linux 表示对tar 目录 进行压缩。
# |compress 表示将压缩文件通过管道操作传递给压缩程序 compres ，并将程序的输出存成 backup.Z文件。
backup: clean
	(cd .. ; tar cf - linux | compress - > backup.Z)
	sync    #使用同步命令迫使缓冲块数据立即写盘并更新超级块。


# 该目标或规则用于产生各个文件之间的依赖关系，创建这些依赖关系是为了让make 目录用来确定是否需要重建一个目标对象。
# 如某个头文件被改动后，make就能通过生成的依赖关系，重新编译与该头文件相关的所有*.c文件。
# 处理过程如下：
# 使用sed字符串编辑程序对makefile进行处理，输出为makefile中删除了'### Dependencies'后面的所有行，
# 并生成一个临时文件 tmp_make,然后对指定目录(init/)的每一C文件执行gcc预处理操作。
# -M 告诉预处理程序cpp输出描述目标文件相关性的规则，并且这些规则符合make语法，对每一个源文件，预处理程序
# 会输出一个规则，其结果形式就是相应源文件的目标文件加上其依赖关系，即该源文件中包含的所有头文件列表。
# 然后把预处理结果都添加到临时文件 tmp_make 中，最后再把这个临时文件复制成新的Makefile文件。
# "$$i" 实际上是'$($i)'
dep:
	sed '/\#\#\# Dependencies/q' < Makefile > tmp_make
	(for i in init/*.c;do echo -n "init/";$(CPP) -M $$i;done) >> tmp_make
	cp tmp_make Makefile
	(cd fs; make dep)    #对fs目录下的makefile也做同样处理，下面类似。
	(cd kernel; make dep)
	(cd mm; make dep)

# main.o 的依赖文件
### Dependencies:
init/main.o : init/main.c include/unistd.h include/sys/stat.h \
  include/sys/types.h include/sys/times.h include/sys/utsname.h \
  include/utime.h include/time.h include/linux/tty.h include/termios.h \
  include/linux/sched.h include/linux/head.h include/linux/fs.h \
  include/linux/mm.h include/signal.h include/asm/system.h include/asm/io.h \
  include/stddef.h include/stdarg.h include/fcntl.h 
