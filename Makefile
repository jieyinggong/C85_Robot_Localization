# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -O2

# Target name
TARGET = ev3_robot

# Source and object files
SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

# Default rule
all: $(TARGET)

# Linking
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

# Compilation rule
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJS) $(TARGET)

# Rebuild rule
rebuild: clean all
