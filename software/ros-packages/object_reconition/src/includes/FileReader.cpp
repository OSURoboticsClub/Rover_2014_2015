/*
 * FileReader:
 * this class will read a file that contains 2 lines in it:
 * line[0] = positive samples
 * line[1] = negative samples
 * all samples need to be divided by spaces, sample names must be absolute paths
 * and be less than 400 chars.
 */
#include "FileReader.h"
using namespace std;

FileReader::FileReader(char *filename){
    myfile.open(filename);
    img = new char[400];
    char_count = 0;
    line_count = 0;
    image_count = 0;
}
/*
 * interates until it finds a space and then returns the word
 * this function will read in a new line if we are at the end of the line when
 * the function is called
 */
char *FileReader::nextimg(){
    if(char_count == line.length()){
        loadline();
    }
    unsigned int len = line.length(); 
    char c = 0;
    int start = char_count;
    while(c != ' ' && char_count < len){
        c = line[char_count];
        char_count++;
        //this deals with any white space at the start of the file
        if(char_count == 1 && c == ' '){
            start++; //just skip the char and move on
            c = 0;//a random char so we dont stop
        }
    }
    //zero the image path to prevent wierd left over chars
    bzero(img, 400);
    line[0] = '\0';
    line.copy(img, char_count-start-1, start); 
    return img;
}

/*
 * Whether or not there is another file to read in
 */
int FileReader::has_next(){
    return !(line_count == 2 && char_count == line.length());
}

/*
 * whether the current image is a positive sample
 */
int FileReader::is_pos(){
    return line_count==1;
}

/*
 * loads the next line if posible
 */
void FileReader::loadline(){
    getline (myfile,line);
    line_count++;
    char_count = 0;
}

/*
 * checks that the file is open and ready to read
 */
int FileReader::is_open(){
    return myfile.is_open();
}
