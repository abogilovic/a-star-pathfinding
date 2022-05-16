#include <iostream>
#include <vector>
#include <list>
#include <math.h>
#include <set>
#include <fstream>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <thread>
#include <chrono>

// A* search

typedef enum
{
    FREE = 0,
    EXPLORED,
    SEEKED,
    OBSTACLE,
    START,
    TARGET
} NodeState;

class Node
{
    public:
        int distances[8] = {10, 14, 10, 14, 10, 14, 10, 14};
        int cost = 0;
        int acost = 0;
        int bcost = 0;
        Node* neighbours[8];
        Node* leastCostNode;

   // private:
        int x, y;
        NodeState state;

    public:
        Node(){}
        Node(int x, int y, NodeState state)
        {
            this->x = x;
            this->y = y;
            this->state = state;
        }


        bool ExploreNode(std::list<Node*> &seekNodes, Node* B)
        {
            if ( this->state == START ) this->acost = 0;
            else this->state = EXPLORED;

            bool updateExploredNeighbours = false;

            for ( int i=0; i<8; ++i )
            {
                if ( neighbours[i] != nullptr )
                {
                    switch (neighbours[i]->state)
                    {
                    case FREE:
                        neighbours[i]->acost = this->acost + distances[i];
                        neighbours[i]->bcost = DistanceTo(B);
                        neighbours[i]->cost = neighbours[i]->acost + neighbours[i]->bcost;

                        neighbours[i]->state = SEEKED;
                        neighbours[i]->leastCostNode = this;

                        seekNodes.push_back(neighbours[i]);
                        //std::cout<<"SEEKED"<<std::endl;
                        break;

                    case SEEKED: // Update seeked
                        if (neighbours[i]->acost > this->acost + distances[i])
                        {
                            //std::cout<<"SEEKED Upd"<<std::endl;
                            neighbours[i]->acost = this->acost + distances[i];
                            neighbours[i]->cost = neighbours[i]->acost + neighbours[i]->bcost;
                            neighbours[i]->leastCostNode = this;
                        }
                        break;

                    case EXPLORED: // Update explored -> seeked
                        if ( neighbours[i]->acost + distances[i] < this->acost )
                        {
                            //std::cout<<"EXPLORED Upd"<<std::endl;
                            this->acost = neighbours[i]->acost + distances[i];
                            this->cost = this->acost + this->bcost;
                            updateExploredNeighbours = true;
                            neighbours[i]->leastCostNode = this;
                        }
                        break;

                    case TARGET:
                        neighbours[i]->leastCostNode = this;
                        return true;
                        break;

                    default:
                        break;
                    }
                }
            }

            if (updateExploredNeighbours)
            {
                for ( int i=0; i<8; ++i )
                {
                    // Update explored -> seeked
                    if ( neighbours[i] != nullptr && neighbours[i]->state == EXPLORED )
                        neighbours[i]->ExploreNode(seekNodes, B);
                }
            }

            return false;
        }


        int DistanceTo(Node * node)
        {
            int d1 = abs(x - node->x);
            int d2 = abs(y - node->y);
            return (14*std::min(d1,d2) + 10*abs(d1-d2));
            return int(sqrt(pow(x-node->x, 2) + pow(y-node->y, 2)));
        }
};



class Astar
{
    public:
        std::list<Node*> seeked;

    Astar(std::vector<std::vector<Node>> &mapa)
    {
        for ( int i=0; i<mapa.size(); ++i )
        {
            for ( int j=0; j<mapa[i].size(); ++j )
            {
                int helper[8][2] = {{i, j+1}, {i-1, j+1}, {i-1, j}, {i-1, j-1}, {i, j-1}, {i+1, j-1}, {i+1, j}, {i+1, j+1}};
                for (int n=0; n<8; ++n)
                {
                    try
                    {
                        mapa.at(i).at(j).neighbours[n] = &(mapa.at(helper[n][0]).at(helper[n][1]));;
                    }
                    catch (const std::out_of_range& oor){}
                }
            }
        }
    }

    void VisualiseSolution(const std::vector<std::vector<Node>> &mapa, Node* N)
    {
        std::vector<Node *> leastNodes;
        while ( N->leastCostNode->state != START )
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            system("clear");

            leastNodes.push_back(N->leastCostNode);

            for (auto row : mapa)
            {
                for (Node node : row)
                {
                    if ( node.state == OBSTACLE )
                        std::cout<<'%';
                    else if ( node.state == START )
                        std::cout<<'A';
                    else if ( node.state == TARGET )
                        std::cout<<'B';
                    else
                    {
                        char c = ' ';
                        for (int i=0; i<leastNodes.size(); ++i)
                        {
                            if ( node.x == leastNodes[i]->x && node.y == leastNodes[i]->y )
                            {
                                c = '@'; break;
                            }
                        }
                        std::cout<<c;
                    }
                }
                std::cout<<'\n';
            }

            N = N->leastCostNode;
        }
        int a; std::cin>>a;
    }

    void VisualiseAlgorithm(const std::vector<std::vector<Node>> &mapa)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        system("clear");

        for (auto row : mapa)
        {
            for (Node node : row)
            {
                if ( node.state == FREE )
                    std::cout<<' ';
                if ( node.state == OBSTACLE )
                    std::cout<<'%';
                else if ( node.state == EXPLORED )
                    std::cout<<'.';
                else if ( node.state == SEEKED )
                    std::cout<<'@';
                else if ( node.state == START )
                    std::cout<<'A';
                else if ( node.state == TARGET )
                    std::cout<<'B';
            }
            std::cout<<'\n';
        }
    }

    void Pathfind(Node* A, Node* B, const std::vector<std::vector<Node>> &mapa)
    {
        seeked.push_back(A);

        while ( 1 )
        {
            VisualiseAlgorithm(mapa);

            auto min_el = std::min_element(seeked.begin(), seeked.end(), [](Node* a, Node* b){
                return a->cost < b->cost;
            });

            //std::cout<<(*min_el)->cost<<" "<<(*min_el)->x<<" "<<(*min_el)->y<<" "<<std::endl;

            if (min_el != seeked.end())
            {
                std::cout<<(*min_el)->cost<<std::endl;
                //std::cout<<(*min_el)->x<< " "<<(*min_el)->y<<std::endl;
                if ((*min_el)->ExploreNode(seeked, B)) break;
                seeked.erase(min_el);
            }
            else{ break; }
        }
    }
};



int main()
{
    std::ifstream  file;
    file.open ("map.txt");
    std::string row;
    std::vector<std::string> lines;
    while( std::getline(file, row) )
    {
        lines.push_back(row);
    }
    file.close();


    std::vector<std::vector<Node>> mapa(lines.size(), std::vector<Node>(lines[0].size()) );
    Node* A; Node* B;

    for ( int i=0; i<lines.size(); ++i )
    {
        for ( int j=0; j<lines[i].size(); ++j )
        {
            switch (lines[i][j])
            {
                case ' ':
                    mapa[i][j] = Node(i, j, FREE);
                    break;
                case '#':
                    mapa[i][j] = Node(i, j, OBSTACLE);
                    break;
                case 'A':
                    mapa[i][j] = Node(i, j, START);
                    A = &mapa[i][j];
                    break;
                case 'B':
                    mapa[i][j] = Node(i, j, TARGET);
                    B = &mapa[i][j];
                    break;
                default:
                    mapa[i][j] = Node(i, j, OBSTACLE);
                    break;
            }
        }
    }

    Astar astar = Astar(mapa);
    std::cin>>row;
    astar.Pathfind(A, B, mapa);
    astar.VisualiseSolution(mapa, B);

    return 0;
}
