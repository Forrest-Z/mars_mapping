#ifndef LINES_RELATION_H
#define LINES_RELATION_H

typedef struct
{
    double intersection_angle;
    double dist;
    double weight;
}LinesRelation;

typedef std::vector<LinesRelation> LinesRelationArray;

#endif 