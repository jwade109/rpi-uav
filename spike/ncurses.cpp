#include <ncurses.h>

int main()
{
    initscr();
    for (uint8_t i = 33; i < 127; i++)
    {
        printw("%c ", i);
        refresh();
    }
    getch();
    endwin();
    return 0;
}
