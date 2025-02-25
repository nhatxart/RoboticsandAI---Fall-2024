//HW2 - Nhat Phan
//Question 1: Describe three hardware components of a robot and what are their functions?
    /* A robot has three main components. The first is the sensors, which are the robot's inputs. These are used to allow the robot to gather
    information regarding its environment. In a software robot, these sensors can track a particular variable of interest. The input data from
    these sensors are then processed by the control system, or processor, of the robot. This component contains programs and instructions
    on how to process the inputted data and make a decision to act on its environment. This action is taken place through effectors, which can
    be actuators that affect physical elements such as a motor generating a force, a heating/cooling module to regulate temperature, or a switch
    that changes the behavior of an electrical circuit. In a software robot, it can be used to change another output variable. */

//Question 2
#include <stdlib.h>
#include <iostream>
#include <bits.h>

using namespace std;

class Node
{
public:
    int data;
    Node* left;
    Node* midleft;
    Node* midright;
    Node* right;
};

//Declare function parameters for printPaths()
void printPathsRecur(Node* Node, int path[], int pathLen);
void printArray(int ints[], int len);

//Recursive helper to print all paths
void printPaths(Node* Node)
{
    int path[1000];
    printPathsRecur(Node, path, 6);
}

//Recursive tool to print path from root to Node
void printPathsRecur(Node* Node, int path[], int pathLen)
{
    if (Node == NULL)
        return;

    //Append this Node to the path array
    path[pathLen] = Node->data;
    pathLen++;

    //Prints path that leads to leaf
    if (Node->left == NULL && Node->right == NULL && Node->midleft == NULL && Node->midright == NULL)
    {
        printArray(path, pathLen);
    }
    else
    {
        //Otherwise try both subtrees
        printPathsRecur(Node->left, path, pathLen);
        printPathsRecur(Node->right, path, pathLen);
        printPathsRecur(Node->midleft, path, pathLen);
        printPathsRecur(Node->midright, path, pathLen);
    }
}


//Prints out arrays
void printArray(int ints[], int len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        cout << ints[i] << " ";
    }
    cout << endl;
}

Node* createNode(int data) {
    Node* newNode = new Node();
    newNode->data = data;
    newNode->left = newNode->right = newNode->midleft = newNode->midright = nullptr;
    return newNode;
};

//Tree code
int main() {
    Node* firstNode = createNode(1); //Orlando is 1
    Node* secondNode = createNode(2); //Tampa is 2
    Node* thirdNode = createNode(3); //Jacksonville is 3
    Node* fourthNode = createNode(4); //Miami is 4
    Node* fifthNode = createNode(5); //Tallahassee is 5

    //Level 1
    firstNode->left = secondNode; //Orlando to Tampa
    firstNode->midleft = thirdNode; //Orlando to Jacksonville
    firstNode->right = fourthNode; //Orlando to Miami

    //Level 2
    firstNode->left->left = fifthNode; //Orlando to Tampa to Tallahassee

    firstNode->left->midleft = firstNode; //Orlando to Tampa to Orlando (terminal)

    firstNode->left->midright = fourthNode; //Orlando to Tampa to Miami
    firstNode->left->right = thirdNode; //Orlando to Tampa to Jacksonville

    firstNode->midleft->left = fifthNode; //Orlando to Jacksonville to Tallahassee

    firstNode->midleft->midleft = firstNode; //Orlando to Jacksonville to Orlando (terminal)

    firstNode->midleft->midright = secondNode; //Orlando to Jacksonville to Tampa
    firstNode->midleft->right = fourthNode; //Orlando to Jacksonville to Miami

    firstNode->right->left = secondNode; //Orlando to Miami to Tampa

    firstNode->right->midleft = firstNode; //Orlando to Miami to Orlando (terminal)
    firstNode->right->midright = nullptr;
    firstNode->right->right = thirdNode; //Orlando to Miami to Jacksonville

    //Level 3
    firstNode->left->left->left = thirdNode; //Orlando to Tampa to Tallahassee to Jacksonville
    firstNode->left->midright->left = thirdNode; //Orlando to Tampa to Miami to Jacksonville

    firstNode->left->midright->right = thirdNode; //Orlando to Tampa to Miami to Orlando (terminal)
    firstNode->left->right->left = fifthNode; //Orlando to Tampa to Jacksonville to Tallahassee (terminal)
    firstNode->left->right->midleft = firstNode; //Orlando to Tampa to Jacksonville to Orlando (terminal)

    firstNode->left->right->right = fourthNode; //Orlando to Tampa to Jacksonville to Miami

    firstNode->midleft->left->left = secondNode; //Orlando to Jacksonville to Tallahassee to Tampa
    firstNode->midleft->midright->left = fifthNode; //Orlando to Jacksonville to Tampa to Tallahassee (terminal because cannot repeat Jacksonville or Tampa)

    firstNode->midleft->midright->right = firstNode; //Orlando to Jacksonville to Tampa to Orlando (terminal)

    firstNode->midleft->right->right = secondNode; //Orlando to Jacksonville to Miami to Tampa

    firstNode->midleft->right->left = firstNode; //Orlando to Jacksonville to Miami to Orlando (terminal)

    firstNode->right->left->left = fifthNode; //Orlando to Miami to Tampa to Tallahassee

    firstNode->right->left->midleft = firstNode; //Orlando to Miami to Tampa to Orlando (terminal)

    firstNode->right->left->right = thirdNode; //Orlando to Miami to Tampa to Jacksonville

    firstNode->right->right->left = fifthNode; //Orlando to Miami to Jacksonville to Tallahassee

    firstNode->right->right->midleft = firstNode; //Orlando to Miami to Jacksonville to Orlando (terminal)

    firstNode->right->right->right = secondNode; //Orlando to Miami to Jacksonville to Tampa

    //Level 4
    firstNode->left->left->left->left = fourthNode; //Orlando to Tampa to Tallahassee to Jacksonville to Miami

    firstNode->left->left->left->right = firstNode; //Orlando to Tampa to Tallahassee to Jacksonville to Orlando (terminal)
    firstNode->left->midright->left->left = fifthNode; //Orlando to Tampa to Miami to Jacksonville to Tallahassee (terminal)
    firstNode->left->midright->left->right = firstNode; //Orlando to Tampa to Miami to Jacksonville to Orlando (terminal)
    firstNode->left->right->right->left = firstNode; //Orlando to Tampa to Jacksonville to Miami to Orlando (terminal)

    firstNode->midleft->left->left->left = fourthNode; //Orlando to Jacksonville to Tallahassee to Tampa to Miami

    firstNode->midleft->left->left->right = firstNode; //Orlando to Jacksonville to Tallahassee to Tampa to Orlando (terminal)
    firstNode->midleft->right->right->left = fifthNode; //Orlando to Jacksonville to Miami to Tampa to Tallahassee (terminal)
    firstNode->midleft->right->right->right = firstNode; //Orlando to Jacksonville to Miami to Tampa to Orlando (terminal)

    firstNode->right->left->left->left = thirdNode; //Orlando to Miami to Tampa to Tallahassee to Jacksonville

    firstNode->right->left->right->left = fifthNode; //Orlando to Miami to Tampa to Jacksonville to Tallahassee (terminal)
    firstNode->right->left->right->right = firstNode; //Orlando to Miami to Tampa to Jacksonville to Orlando (terminal)

    firstNode->right->right->left->left = secondNode; //Orlando to Miami to Jacksonville to Tallahassee to Tampa

    firstNode->right->right->right->left = fifthNode; //Orlando to Miami to Jacksonville to Tampa to Tallahassee (terminal)
    firstNode->right->right->right->right = firstNode; //Orlando to Miami to Jacksonville to Tampa to Orlando (terminal)

    //Level 5
    firstNode->left->left->left->left->left = firstNode; //Orlando to Tampa to Tallahassee to Jacksonville to Miami to Orlando (completed)

    firstNode->midleft->left->left->left->left = firstNode; //Orlando to Jacksonville to Tallahassee to Tampa to Miami to Orlando (completed)

    firstNode->right->left->left->left->left = firstNode; //Orlando to Miami to Tampa to Tallahassee to Jacksonville to Orlando (completed)
    firstNode->right->right->left->left->left = firstNode; //Orlando to Miami to Jacksonville to Tallahassee to Tampa to Orlando (completed)

    printPaths(firstNode);
    return 0;
};