#!
make -j  isoimage && qemu-system-x86_64 -smp 1 -m 2048 -vga std -serial stdio -cdrom nautilus.iso -gdb tcp::1234 -netdev tap,id=vm0,ifname=mytap0,script=no,downscript=no -device rtl8139,netdev=vm0,mac=52:00:01:02:03:04
