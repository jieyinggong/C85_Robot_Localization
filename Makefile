# Compiler and flags
CC = g++
CFLAGS = -Wall -Wextra -O2 -I"EV3_RobotControl"

# Target name
TARGET = ev3_robot

# Source and object files
SRCS = $(wildcard *.c EV3_RobotControl/btcomm.c)
OBJS = $(SRCS:.c=.o)

# Default rule
all: $(TARGET)

# Linking
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ -lbluetooth

# Compilation rule
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ 

# Clean rule
clean:
	rm -f $(OBJS) $(TARGET)

# Rebuild rule
rebuild: clean all
