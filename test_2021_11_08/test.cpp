#include<iostream>

int main(int argc, char* argv[]){
    if(argc != 2){
        std::cout << "Please input your name, Thanks" << std::endl;
    }
    else{
        std::cout << "Hello, " << argv[1] << std::endl;
    }
}