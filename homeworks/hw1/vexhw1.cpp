#include <iostream>
#include <tgmath.h>
using namespace std;

float add(float, float);
float subtract(float, float);
float multiply(float, float);
float divide(float, float);
float square(float);
float sqroot(float);

int main(){
    string input;
    float num1, num2, out;
    string operation;
    
    cout << "Enter first number: ";
        cin >> num1;
    
    cout << "Operation inputs:\nadd, sub, mult, \ndiv, square, sqrt\n";
        cout << "Enter operation: ";
        cin >> input;
    while(input != "add" && input != "sub" && input != "mult" &&
          input != "div" && input != "square" && input != "sqrt"){
        cout << "Invalid Opertion.\nEnter operation: ";
        cin >> input;
    }
    if(input != "square" && input != "sqrt"){
        cout << "Enter second number: ";
        cin >> num2;
    } 
    
    if(input == "add"){
        out = add(num1, num2);
    }
    else if(input == "sub"){
        out = subtract(num1, num2);
    }
    else if(input == "mult"){
        out = multiply(num1, num2);
    }
    else if(input == "div"){
        out = divide(num1, num2);
    }
    else if(input == "square"){
        out = square(num1);
    }
    else if(input == "sqrt"){
        out = sqroot(num1);
    }
    cout << out << "\n";
    return 0;
}


float add(float num1, float num2){
    return num1 + num2;
}

float subtract(float num1, float num2){
    return num1 - num2;
}

float multiply(float num1, float num2){
    return num1 * num2;
}

float divide(float num1, float num2){
    return num1 / num2;
}

float square(float num1){
    return num1 * num1;
}

float sqroot(float num1){
    return sqrtf(num1);
}
