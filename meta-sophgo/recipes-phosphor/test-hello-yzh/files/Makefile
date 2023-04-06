TARGET=testhelloyzh
LDFLAGS= -lpthread

#获取当前目录下的c文件集
CUR_SOURCE=${wildcard *.c}
#将对应的c文件名转为o文件后放在下面的CUR_OBJS变量中
    CUR_OBJS=${patsubst %.c, %.o, $(CUR_SOURCE)}
all: $(TARGET)
$(TARGET): $(CUR_OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
clean:
	rm -f $(CUR_OBJS) $(TARGET)
