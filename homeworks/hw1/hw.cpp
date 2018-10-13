#include <iostream>
#include <cmath>
using namespace std;

float add(float val1, float val2)
{
	return val1 + val2;
}

float subtract(float val1, float val2)
{
	return val1 - val2;
}
float multiply(float val1, float val2)
{
	return val1 * val2;
}
float divide(float val1, float val2)
{
	return val1 / val2;
}
float square(float val1)
{
	return pow(val1, 2);
}
int main()
{
	float parameter1, parameter2;
	string operation;
	cout <<"What operation? add subtract multiply divide square sqrt"<< endl;
	cin >> operation;
	if (operation == "square" or operation == "sqrt")
	{
		cout << "Value?" << endl;
		cin >> parameter1;
	}
	else
	{
		cout  << "Value 1?" << endl;
		cin >> parameter1;
		cout << "Value 2?" << endl;
		cin >> parameter2;
	}
	if (operation == "add"){
		cout << add(parameter1, parameter2);
	}
	else if (operation == "subtract")
	{
		cout << subtract(parameter1, parameter2);
	}
	else if (operation == "multiply")
	{
		cout << multiply(parameter1, parameter2);
	}
	else if (operation == "divide")
	{
		cout << divide(parameter1, parameter2);
	}
	else if (operation == "square")
	{
		cout << square(parameter1);
	}
	else if (operation == "sqrt")
	{
		cout << sqrt(parameter1);
	}
}


