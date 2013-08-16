all: ikfast.HIRO_RARM.so ikfast.HIRO_LARM.so

ikfast.HIRO_RARM.o ikfast.HIRO_LARM.o:
	g++ -fPIC -c -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY $(*F).cpp


ikfast.HIRO_RARM.so: ikfast.HIRO_RARM.o
	g++ -shared -Wl,-soname,ikfast.HIRO_RARM.so ikfast.HIRO_RARM.o -llapack -lstdc++ -o ikfast.HIRO_RARM.so

ikfast.HIRO_LARM.so: ikfast.HIRO_LARM.o
	g++ -shared -Wl,-soname,ikfast.HIRO_LARM.so ikfast.HIRO_LARM.o -llapack -lstdc++ -o ikfast.HIRO_LARM.so

clean:
	rm -f *.o *~ *.log *.so
