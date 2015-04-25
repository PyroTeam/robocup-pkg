#include "Modele.h"
#include "Segment.h"
#include "Machine.h"

#include <cmath>
#include <stdio.h>
#include <stdlib.h>

#include "line_detection_utils.h"

Segment build(const std::list<Point> &points);

std::list<Segment> buildSegment(Modele m, double seuil);

std::list<Segment> buildSegments(std::list<Modele> &listOfModeles);

std::list<Machine> recognizeMachinesFrom(std::list<Segment> listOfSegments);