CC ?= cc
CFLAGS ?= -std=c11 -O2 -Wall -Wextra -Wpedantic -Werror
INC = -I../../Inc
SRC = test_eskf.c ../../Src/eskf15.c ../../Src/eskf_math.c ../../Src/eskf_health.c ../../Src/app_eskf.c

.PHONY: test clean
test: test_eskf
	./test_eskf

test_eskf: $(SRC)
	$(CC) $(CFLAGS) $(INC) $(SRC) -lm -o $@

clean:
	$(RM) test_eskf
