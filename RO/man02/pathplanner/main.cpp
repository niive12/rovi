#include <iostream>
#include <fstream>

int main(){
    std::ofstream out("out.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!

    std::cout << "Hello World!\n";

    std::cout.rdbuf(coutbuf); //reset to standard output again
    return 0;
}

