CFLAGS  += -Wall -O3
CFLAGS  += `ncurses5-config --cflags`
LDFLAGS += `ncurses5-config --libs`

all: ScoobyCAN ScoobyCAN_dump tags

ScoobyCAN_dump: ScoobyCAN.c
	gcc         -DTPM_STEER_LIMIT=1000 -DTPM_COUNT_LIMIT=2000 -Wall -O3 $< -lncurses -ltinfo -o $@

ScoobyCAN: ScoobyCAN.c
	gcc -DNCURS -DTPM_STEER_LIMIT=180 -DTPM_COUNT_LIMIT=2 -Wall -O3 $< -lncurses -ltinfo -o $@

tags:
	ctags -R *

clean:
	rm ScoobyCAN ScoobyCAN_dump ScoobyCAN.o
