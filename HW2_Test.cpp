#include <stdlib.h>
#include <iostream>
using namespace std;

struct Node {
    int data;
    Node* left;
    Node* midleft;
    Node* midright;
    Node* right;
};
Node* createNode(int data) {
    Node* newNode = new Node();
    newNode->data = data;
    newNode->left = newNode->right = newNode->midleft = newNode->midright = nullptr;
    return newNode;
};

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
}
//    //Level 2
//    firstNode->left->left = fifthNode; //Orlando to Tampa to Tallahassee
//
//    firstNode->left->midleft = firstNode; //Orlando to Tampa to Orlando (terminal)
//
//    firstNode->left->midright = fourthNode; //Orlando to Tampa to Miami
//    firstNode->left->right = thirdNode; //Orlando to Tampa to Jacksonville
//
//    firstNode->midleft->left = fifthNode; //Orlando to Jacksonville to Tallahassee
//
//    firstNode->midleft->midleft = firstNode; //Orlando to Jacksonville to Orlando (terminal)
//
//    firstNode->midleft->midright = secondNode; //Orlando to Jacksonville to Tampa
//    firstNode->midleft->right = fourthNode; //Orlando to Jacksonville to Miami
//
//    firstNode->right->left = secondNode; //Orlando to Miami to Tampa
//
//    firstNode->right->midleft = firstNode; //Orlando to Miami to Orlando (terminal)
//    firstNode->right->midright = nullptr;
//    firstNode->right->right = thirdNode; //Orlando to Miami to Jacksonville
//
//    //Level 3
//    firstNode->left->left->left = thirdNode; //Orlando to Tampa to Tallahassee to Jacksonville
//    firstNode->left->midright->left = thirdNode; //Orlando to Tampa to Miami to Jacksonville
//
//    firstNode->left->midright->right = thirdNode; //Orlando to Tampa to Miami to Orlando (terminal)
//    firstNode->left->right->left = fifthNode; //Orlando to Tampa to Jacksonville to Tallahassee (terminal)
//    firstNode->left->right->midleft = firstNode; //Orlando to Tampa to Jacksonville to Orlando (terminal)
//
//    firstNode->left->right->right = fourthNode; //Orlando to Tampa to Jacksonville to Miami
//
//    firstNode->midleft->left->left = secondNode; //Orlando to Jacksonville to Tallahassee to Tampa
//    firstNode->midleft->midright->left = fifthNode; //Orlando to Jacksonville to Tampa to Tallahassee (terminal because cannot repeat Jacksonville or Tampa)
//
//    firstNode->midleft->midright->right = firstNode; //Orlando to Jacksonville to Tampa to Orlando (terminal)
//
//    firstNode->midleft->right->right = secondNode; //Orlando to Jacksonville to Miami to Tampa
//
//    firstNode->midleft->right->left = firstNode; //Orlando to Jacksonville to Miami to Orlando (terminal)
//
//    firstNode->right->left->left = fifthNode; //Orlando to Miami to Tampa to Tallahassee
//
//    firstNode->right->left->midleft = firstNode; //Orlando to Miami to Tampa to Orlando (terminal)
//
//    firstNode->right->left->right = thirdNode; //Orlando to Miami to Tampa to Jacksonville
//
//    firstNode->right->right->left = fifthNode; //Orlando to Miami to Jacksonville to Tallahassee
//
//    firstNode->right->right->midleft = firstNode; //Orlando to Miami to Jacksonville to Orlando (terminal)
//
//    firstNode->right->right->right = secondNode; //Orlando to Miami to Jacksonville to Tampa
//
//    //Level 4
//    firstNode->left->left->left->left = fourthNode; //Orlando to Tampa to Tallahassee to Jacksonville to Miami
//
//    firstNode->left->left->left->right = firstNode; //Orlando to Tampa to Tallahassee to Jacksonville to Orlando (terminal)
//    firstNode->left->midright->left->left = fifthNode; //Orlando to Tampa to Miami to Jacksonville to Tallahassee (terminal)
//    firstNode->left->midright->left->right = firstNode; //Orlando to Tampa to Miami to Jacksonville to Orlando (terminal)
//    firstNode->left->right->right->left = firstNode; //Orlando to Tampa to Jacksonville to Miami to Orlando (terminal)
//
//    firstNode->midleft->left->left->left = fourthNode; //Orlando to Jacksonville to Tallahassee to Tampa to Miami
//
//    firstNode->midleft->left->left->right = firstNode; //Orlando to Jacksonville to Tallahassee to Tampa to Orlando (terminal)
//    firstNode->midleft->right->right->left = fifthNode; //Orlando to Jacksonville to Miami to Tampa to Tallahassee (terminal)
//    firstNode->midleft->right->right->right = firstNode; //Orlando to Jacksonville to Miami to Tampa to Orlando (terminal)
//
//    firstNode->right->left->left->left = thirdNode; //Orlando to Miami to Tampa to Tallahassee to Jacksonville
//
//    firstNode->right->left->right->left = fifthNode; //Orlando to Miami to Tampa to Jacksonville to Tallahassee (terminal)
//    firstNode->right->left->right->right = firstNode; //Orlando to Miami to Tampa to Jacksonville to Orlando (terminal)
//
//    firstNode->right->right->left->left = secondNode; //Orlando to Miami to Jacksonville to Tallahassee to Tampa
//
//    firstNode->right->right->right->left = fifthNode; //Orlando to Miami to Jacksonville to Tampa to Tallahassee (terminal)
//    firstNode->right->right->right->right = firstNode; //Orlando to Miami to Jacksonville to Tampa to Orlando (terminal)
//
//    //Level 5
//    firstNode->left->left->left->left->left = firstNode; //Orlando to Tampa to Tallahassee to Jacksonville to Miami to Orlando (completed)
//
//    firstNode->midleft->left->left->left->left = firstNode; //Orlando to Jacksonville to Tallahassee to Tampa to Miami to Orlando (completed)
//
//    firstNode->right->left->left->left->left = firstNode; //Orlando to Miami to Tampa to Tallahassee to Jacksonville to Orlando (completed)
//    firstNode->right->right->left->left->left = firstNode; //Orlando to Miami to Jacksonville to Tallahassee to Tampa to Orlando (completed)
//
//    return 0;
//}