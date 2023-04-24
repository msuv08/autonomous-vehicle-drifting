#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

// Define a Node structure for the circular linked list
struct Node {
    vector<double> data;
    Node* next;
};

int main() {
    // Open the CSV file for reading
    ifstream file("data.csv");
    if (!file.is_open()) {
        cerr << "Error: could not open file.\n";
        return 1;
    }

    // Read the CSV file line by line and create a circular linked list of nodes
    Node* head = nullptr;
    Node* tail = nullptr;
    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string field;
        getline(ss, field, ',');  // Ignore the first element of the record
        getline(ss, field, ',');  // Get the second element of the record
        vector<double> value;
        string::size_type start = field.find_first_of("[");
        string::size_type end = field.find_last_of("]");
        if (start != string::npos && end != string::npos) {
            string str = field.substr(start+1, end-start-1);
            stringstream ss2(str);
            double num;
            while (ss2 >> num) {
                value.push_back(num);
                if (ss2.peek() == ',') {
                    ss2.ignore();
                }
            }
        }
        Node* node = new Node{value, nullptr};
        if (tail == nullptr) {
            head = node;
            tail = node;
        } else {
            tail->next = node;
            tail = node;
        }
    }
    // Make the linked list circular
    tail->next = head;

    // Traverse the linked list and print the values of each node
    Node* current = head;
    do {
        cout << "[";
        for (size_t i = 0; i < current->data.size(); ++i) {
            cout << current->data[i];
            if (i < current->data.size() - 1) {
                cout << ", ";
            }
        }
        cout << "]" << endl;
        current = current->next;
    } while (current != head);

    // Free the memory used by the nodes
    current = head;
    do {
        Node* next = current->next;
        delete current;
        current = next;
    } while (current != head);

    return 0;
}
