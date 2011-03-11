#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <strings.h>
#include <ply.h>


/* user's vertex and face definitions for a polygonal object */

typedef struct Vertex {
  float x,y,z;
  float nx,ny,nz;
  void *other_props;       /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
  void *other_props;       /* other properties */
} Face;

char *elem_names[] = { /* list of the kinds of elements in the user's object */
  "vertex", "face"
};

PlyProperty vert_props[] = { /* list of property information for a vertex */
  {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
  {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
  {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
  {"nx", Float32, Float32, offsetof(Vertex,nx), 0, 0, 0, 0},
  {"ny", Float32, Float32, offsetof(Vertex,ny), 0, 0, 0, 0},
  {"nz", Float32, Float32, offsetof(Vertex,nz), 0, 0, 0, 0},
};

PlyProperty face_props[] = { /* list of property information for a face */
  {"vertex_indices", Int32, Int32, offsetof(Face,verts),
   1, Uint8, Uint8, offsetof(Face,nverts)},
};


/*** the PLY object ***/

static PlyFile *in_ply;
static int nverts,nfaces;
static Vertex **vlist;
static Face **flist;
static PlyOtherProp *vert_other,*face_other;

static int flip_order = 1;       /* flip order of vertices around the faces? */
static int flip_normals = 0;     /* flip vertex normals? */

/* are normals in PLY file? */
static int has_nx = 0;
static int has_ny = 0;
static int has_nz = 0;


/******************************************************************************
Main program.
******************************************************************************/

main(int argc, char *argv[])
{
  read_file();

  generate_histogram();

  write_histogram();
}


/******************************************************************************
Read in the PLY file from standard in.
******************************************************************************/

read_file()
{
  int i,j;
  int elem_count;
  char *elem_name;

  /*** Read in the original PLY object ***/

  in_ply = read_ply (stdin);

  /* examine each element type that is in the file (vertex, face) */

  for (i = 0; i < in_ply->num_elem_types; i++) {

    /* prepare to read the i'th list of elements */
    elem_name = setup_element_read_ply (in_ply, i, &elem_count);

    if (equal_strings ("vertex", elem_name)) {

      /* create a vertex list to hold all the vertices */
      vlist = (Vertex **) malloc (sizeof (Vertex *) * elem_count);
      nverts = elem_count;

      /* set up for getting vertex elements */
      /* (we want x,y,z) */

      setup_property_ply (in_ply, &vert_props[0]);
      setup_property_ply (in_ply, &vert_props[1]);
      setup_property_ply (in_ply, &vert_props[2]);

      /* we also want normal information if it is there (nx,ny,nz) */

      for (j = 0; j < in_ply->elems[i]->nprops; j++) {
	PlyProperty *prop;
	prop = in_ply->elems[i]->props[j];
	if (equal_strings ("nx", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[3]);
	  has_nx = 1;
	}
	if (equal_strings ("ny", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[4]);
	  has_ny = 1;
	}
	if (equal_strings ("nz", prop->name)) {
	  setup_property_ply (in_ply, &vert_props[5]);
	  has_nz = 1;
	}
      }

      /* also grab anything else that we don't need to know about */

      vert_other = get_other_properties_ply (in_ply, 
					     offsetof(Vertex,other_props));

      /* grab the vertex elements and store them in our list */

      for (j = 0; j < elem_count; j++) {
        vlist[j] = (Vertex *) malloc (sizeof (Vertex));
        get_element_ply (in_ply, (void *) vlist[j]);
      }
    }
    else if (equal_strings ("face", elem_name)) {

      /* create a list to hold all the face elements */
      flist = (Face **) malloc (sizeof (Face *) * elem_count);
      nfaces = elem_count;

      /* set up for getting face elements */
      /* (all we need are vertex indices) */

      setup_property_ply (in_ply, &face_props[0]);
      face_other = get_other_properties_ply (in_ply, 
					     offsetof(Face,other_props));

      /* grab all the face elements and place them in our list */

      for (j = 0; j < elem_count; j++) {
        flist[j] = (Face *) malloc (sizeof (Face));
        get_element_ply (in_ply, (void *) flist[j]);
      }
    }
    else  /* all non-vertex and non-face elements are grabbed here */
      get_other_element_ply (in_ply);
  }

  /* close the file */
  /* (we won't free up the memory for in_ply because we will use it */
  /*  to help describe the file that we will write out) */

  close_ply (in_ply);
}


generate_histogram()
{
  int i;
  for (i = 0; i < nverts; ++i)
  {
    if (has_nx && has_ny && has_nz)
    {
      //update
    }
  }
}


/******************************************************************************
Write out the PLY file to standard out.
******************************************************************************/

write_histogram()
{
}

