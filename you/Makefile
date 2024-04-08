SRCDIR := src
OBJDIR := obj
SRCFILES := $(wildcard $(SRCDIR)/*.c)
OBJFILES := $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(SRCFILES))

SRCEXTDIR := src_ext
SRCEXTFILES := $(wildcard $(SRCEXTDIR)/*.c)

INCLUDEDIR := -I$(SRCDIR) -I$(SRCEXTDIR)

CFLAGS := -std=gnu99 -g 
LFLAGS := -lX11 -lm 

BIN := you

all: $(BIN)

$(BIN): $(OBJFILES) 
	gcc -o $@ $^ $(CFLAGS) $(LFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.c
	gcc -o $@ $< -c $(CFLAGS) $(INCLUDEDIR)
	
$(OBJEXTDIR)/%.o: $(SRCEXTDIR)/%.c
	gcc -o $@ $< -c $(CFLAGS) $(INCLUDEDIR)

clean: 
	rm $(OBJFILES)
	rm $(OBJEXTFILES)
	rm $(BIN)
