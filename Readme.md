##qemu-mini2440 
修改过的qqmu-mini2440，原来的git，http://repo.or.cz/w/qemu/mini2440.git 
无法正确编译，修改了部分文件，能够正常使用 

##编译说明 
./configure --target-list=arm-softmmu --prefix=$HOME/local 
make && make install 


