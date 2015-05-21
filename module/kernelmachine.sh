qemu-system-x86_64 -readconfig /home/tim/VMs/kernel.config -smb /home/tim/src -smp 2 -m 1G -netdev user,id=mynet0,hostfwd="tcp::5555-:22" &

nmap localhost -p5555 -sV | grep ssh;  
while [ $? -ne 0 ]; do !!; done

ssh  localhost -p 5555 -t "cd ~/src/rtlsdrng/module; make clean; make; sudo rmmod rtlsdr.ko -f ; sudo insmod rtlsdr.ko; dmesg -w"
#ssh  localhost -p 5555 -t "sudo shutdown -h now"
