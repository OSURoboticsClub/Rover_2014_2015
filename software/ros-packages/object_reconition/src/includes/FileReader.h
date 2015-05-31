/*
 * FileReader:
 * this class will read a file that contains 2 lines in it:
 * line[0] = positive samples
 * line[1] = negative samples
 * all samples need to be divided by spaces, sample names must be absolute paths
 * and be less than 400 chars.
 */
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;

class FileReader{
    private:
        int line_count;
        string line;
        int char_count;
        int image_count;
        char *img;
        ifstream myfile;
        char *directory;
        void loadline();
    public:
        FileReader(char *filename);
        char *nextimg();
        int is_open();
        int has_next();
        int is_pos();
};

