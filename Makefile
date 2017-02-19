CFLAGS  += -Wall -O3
CFLAGS  += `ncurses5-config --cflags`
LDFLAGS += `ncurses5-config --libs`

all: ScoobyCAN ScoobyCAN_dump tags

ScoobyCAN_dump: ScoobyCAN.c
	gcc         -DTPMS_STEER_LIMIT=0 -DTPMS_COUNT_LIMIT=20000 $(CFLAGS) $< $(LDFLAGS) -o $@

ScoobyCAN: ScoobyCAN.c
	gcc -DNCURS -DTPMS_STEER_LIMIT=5 -DTPMS_COUNT_LIMIT=500   $(CFLAGS) $< $(LDFLAGS) -o $@

tags:
	ctags -R *

clean:
	rm ScoobyCAN ScoobyCAN_dump ScoobyCAN.o
