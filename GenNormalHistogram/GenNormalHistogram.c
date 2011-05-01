#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ply.h>

#define PI 3.14159

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

double histogram[120][30];
double bx1, by1, bz1, bx2, by2, bz2, bx3, by3, bz3;
/******************************************************************************
Main program.
******************************************************************************/

main(int argc, char *argv[])
{
  read_file();

  generate_histogram();
  compute_axis();
  project_onto_basis();
  print_basis();
  write_file();
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

project_onto_basis()
{
  int i;
  double projections [6] = {by1, -by1, by2, -by2, by3, -by3};
  double maxValue = 0;
  int maxIndex = 0;
  for (i = 0; i < 6; ++i)
  {
    if (maxValue <= projections[i])
    {
      maxValue = projections[i];
      maxIndex = i;
    }
  }
  double y [3];
  switch (maxIndex)
  {
    case 0:
      y[0] = 1;
      y[1] = 0;
      y[2] = 0;
      break;
    case 1:
      y[0] = -1;
      y[1] = 0;
      y[2] = 0;
      break;
    case 2:
      y[0] = 0;
      y[1] = 1;
      y[2] = 0;
      break;
    case 3:
      y[0] = 0;
      y[1] = -1;
      y[2] = 0;
      break;
    case 4:
      y[0] = 0;
      y[1] = 0;
      y[2] = 1;
      break;
    case 5:
      y[0] = 0;
      y[1] = 0;
      y[2] = -1;
      break;
  }
  double up[3] = {0, 1, 0};
  double axis[3] = {y[2]*up[1] - y[1]*up[2], y[0]*up[2] - y[2]*up[0], y[1]*up[0] - y[0]*up[1]};
  double angle = acos(y[0]*up[0] + y[1]*up[1] + y[2]*up[2]);
  double R [9] = {0,0,0,0,0,0,0,0,0};
  R[0] = cos(angle);
  R[4] = cos(angle);
  R[8] = cos(angle);

  double E[9] = {axis[0]*axis[0], axis[0]*axis[1], axis[0]*axis[2],
                 axis[1]*axis[0], axis[1]*axis[1], axis[1]*axis[2],
                 axis[2]*axis[0], axis[2]*axis[1], axis[2]*axis[2]};
  for (i = 0; i < 9; ++i)
  {
    R[i] += (1 - cos(angle))*E[i];
  }
  R[1] -= sin(angle)*axis[2];
  R[2] += sin(angle)*axis[1];
  R[3] += sin(angle)*axis[2];
  R[5] -= sin(angle)*axis[0];
  R[6] -= sin(angle)*axis[1];
  R[7] += sin(angle)*axis[0];
  for (i = 0; i < nverts; ++i)
  {
    double x, y, z, tempx, tempy, tempz;
    if (has_nx && has_ny && has_nz)
    {
      tempx = bx1*vlist[i]->nx + by1*vlist[i]->ny + bz1*vlist[i]->nz;
      tempy = bx2*vlist[i]->nx + by2*vlist[i]->ny + bz2*vlist[i]->nz;
      tempz = bx3*vlist[i]->nx + by3*vlist[i]->ny + bz3*vlist[i]->nz;
      x = R[0]*tempx + R[3]*tempy + R[6]*tempz;
      y = R[1]*tempx + R[4]*tempy + R[7]*tempz;
      z = R[2]*tempx + R[5]*tempy + R[8]*tempz;
      vlist[i]->nx = x;
      vlist[i]->ny = y;
      vlist[i]->nz = z;
    }
    tempx = bx1*vlist[i]->x + by1*vlist[i]->y + bz1*vlist[i]->z;
    tempy = bx2*vlist[i]->x + by2*vlist[i]->y + bz2*vlist[i]->z;
    tempz = bx3*vlist[i]->x + by3*vlist[i]->y + bz3*vlist[i]->z;
    x = R[0]*tempx + R[3]*tempy + R[6]*tempz;
    y = R[1]*tempx + R[4]*tempy + R[7]*tempz;
    z = R[2]*tempx + R[5]*tempy + R[8]*tempz;
    vlist[i]->x = x;
    vlist[i]->y = y;
    vlist[i]->z = z;

  }
}

print_basis()
{
  fprintf (stderr, "%f %f %f\n", bx1, by1, bz1);
  fprintf (stderr, "%f %f %f\n", bx2, by2, bz2);
  fprintf (stderr, "%f %f %f\n", bx3, by3, bz3);
  double angle1 = 180*acos(bx1*bx2 + by1*by2 + bz1*bz2)/PI;
  double angle2 = 180*acos(bx2*bx3 + by2*by3 + bz2*bz3)/PI;
  double angle3 = 180*acos(bx3*bx1 + by3*by1 + bz3*bz1)/PI;
  fprintf (stderr, "%lf %lf %lf\n", angle1, angle2, angle3);
  fprintf (stderr, "%f %f\n", 60*acos(bz1)/PI, 60*(atan2(by1, bx1))/PI);
  fprintf (stderr, "%f %f\n", 60*acos(bz2)/PI, 60*(atan2(by2, bx2))/PI);
  fprintf (stderr, "%f %f\n", 60*acos(bz3)/PI, 60*(atan2(by3, bx3))/PI);
}

generate_histogram()
{
  int i, j;
  for (i = 0; i < 120; ++i)
  {
    for (j = 0; j < 30; ++j)
    {
      histogram[i][j] = 0;
    }
  }
  for (i = 0; i < nverts; ++i)
  {
    if (has_nx && has_ny && has_nz)
    {
      double x = vlist[i]->nx;
      double y = vlist[i]->ny;
      double z = vlist[i]->nz;
      double r = sqrt(x*x + y*y + z*z);
      if (y < 0)
      {
        r *= -1;
      }
      if (r == 0)
      {
        continue;
      }
      x /= r;
      y /= r;
      z /= r;
      int theta = 60*acos(z)/PI;
      int phi = 60*(atan2(y, x) + PI)/PI;
      histogram[theta][phi]++;
    }
  }
  for (i = 0; i < 120; ++i)
  {
    for (j = 0; j < 30; ++j)
    {
      double theta_0 = PI*i/60;
      double phi_0 = PI*j/60;
      double delta_phi = PI/60;
      double delta_theta = PI/60;
      double normalization = delta_theta*(cos(phi_0) - cos(phi_0 + delta_phi));
      histogram[i][j] /= normalization;
    }
  }
}


compute_axis()
{
  int i, j, max = 0, theta_max = 0, phi_max = 0;
  for (i = 0; i < 120; ++i)
  {
    for (j = 0; j < 30; ++j)
    {
      if (histogram[i][j] >= max)
      {
        max = histogram[i][j];
        theta_max = i;
        phi_max = j;
      }
    }
  }
  bx1 = sin(PI*theta_max/60)*cos(PI*phi_max/60);
  by1 = sin(PI*theta_max/60)*sin(PI*phi_max/60);
  bz1 = cos(PI*theta_max/60);
  max = 0;
  for (i = 0; i < 120; ++i)
  {
    for (j = 0; j < 30; ++j)
    {
      double xtemp = sin(PI*i/60)*cos(PI*j/60);
      double ytemp = sin(PI*i/60)*sin(PI*j/60);
      double ztemp = cos(PI*i/60);
      double angle = acos(xtemp*bx1 + ytemp*by1 + ztemp*bz1);
      if (angle > PI/2 - 0.08 && angle < PI/2 + 0.08 && histogram[i][j] >= max)
      {
        max = histogram[i][j];
        bx2 = xtemp;
        by2 = ytemp;
        bz2 = ztemp;
      }
    }
  }
  max = 0;
  for (i = 0; i < 120; ++i)
  {
    for (j = 0; j < 30; ++j)
    {
      double xtemp = sin(PI*i/60)*cos(PI*j/60);
      double ytemp = sin(PI*i/60)*sin(PI*j/60);
      double ztemp = cos(PI*i/60);
      double angle1 = acos(xtemp*bx1 + ytemp*by1 + ztemp*bz1);
      double angle2 = acos(xtemp*bx2 + ytemp*by2 + ztemp*bz2);
      if (angle1 > PI/2 - 0.08 && angle1 < PI/2 + 0.08 && angle2 > PI/2 - 0.08 && angle2 < PI/2 + 0.08 && histogram[i][j] >= max)
      {
        max = histogram[i][j];
        bx3 = xtemp;
        by3 = ytemp;
        bz3 = ztemp;
      }
    }
  }
  for (i = 0; i < 120; ++i)
  {
    for (j = 0; j < 30; ++j)
    {
      fprintf (stderr, "%lf ", histogram[i][j]);
    }
    fprintf (stderr, "\n");
  }

}


/******************************************************************************
Write out the PLY file to standard out.
******************************************************************************/

write_file()
{
  int i;
  PlyFile *ply;
  char **elist;
  int num_elem_types;

  /*** Write out the transformed PLY object ***/

  elist = get_element_list_ply (in_ply, &num_elem_types);
  ply = write_ply (stdout, num_elem_types, elist, in_ply->file_type);

  /* describe what properties go into the vertex elements */
  /* (position x,y,z and normals nx,ny,nz if they were provided) */

  describe_element_ply (ply, "vertex", nverts);
  describe_property_ply (ply, &vert_props[0]);
  describe_property_ply (ply, &vert_props[1]);
  describe_property_ply (ply, &vert_props[2]);
  if (has_nx) describe_property_ply (ply, &vert_props[3]);
  if (has_ny) describe_property_ply (ply, &vert_props[4]);
  if (has_nz) describe_property_ply (ply, &vert_props[5]);

  /* all other vertex properties besides position and normal */
  describe_other_properties_ply (ply, vert_other, offsetof(Vertex,other_props));

  /* describe face properties (just list of vertex indices) */
  describe_element_ply (ply, "face", nfaces);
  describe_property_ply (ply, &face_props[0]);
  describe_other_properties_ply (ply, face_other, offsetof(Face,other_props));

  /* all other properties that we tucked away are mentioned here */
  describe_other_elements_ply (ply, in_ply->other_elems);

  /* copy the comments and other textual object information */
  copy_comments_ply (ply, in_ply);
  append_comment_ply (ply, "modified by flipply");
  copy_obj_info_ply (ply, in_ply);

  /* we've told the routines enough information so that the file header */
  /* can be written out now */
  header_complete_ply (ply);

  /* set up and write the vertex elements */
  put_element_setup_ply (ply, "vertex");
  for (i = 0; i < nverts; i++)
    put_element_ply (ply, (void *) vlist[i]);
  
  /* set up and write the face elements */
  put_element_setup_ply (ply, "face");
  for (i = 0; i < nfaces; i++)
    put_element_ply (ply, (void *) flist[i]);
  
  /* the other properties that we tucked away are written out here */
  put_other_elements_ply (ply);
  
  /* close the file and free up the memory */
  
  close_ply (ply);
  free_ply (ply);
} 

