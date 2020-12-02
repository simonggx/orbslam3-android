#include "TemplatedVocabulary.h"
#include "FORB.h"

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;
int main(int argc, char *argv[])
{
    ORBVocabulary orbVocabulary;
    orbVocabulary.loadFromTextFile(argv[1]);
    orbVocabulary.saveToBinaryFile(argv[2]);
    orbVocabulary.loadFromBinaryFile(argv[2]);
    return 0;
}