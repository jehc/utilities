#include <iostream>

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " [input].dae" << std::endl;
  }
  DAE dae;
  daeElement* root = dae.open(argv[1]);
  if (!root)
  {
    std::cout << "Could not open file " << argv[1] << std::endl;
    return -1;
  }
  daeDatabase * database = dae.getDatabase();
  if (!database)
  {
    std::cout << "Failed to construct database" << std::endl;
    return -1;
  }
  std::vector<domNode*> nodes = database->typeLookup<domNode>();
  std::vector<domNode*> geometries;
  for (size_t i = 0; i < nodes.size(); ++i)
  {
    if (nodes[i]->getId() == "geometry")
    {
      geometries.push_back (nodes[i]);
    }
  }
  
  return 0;
}
