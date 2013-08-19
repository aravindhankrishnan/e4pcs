#include "config.h"

#include <iostream>
using namespace std;

namespace E4PCS
{

InputParamsPtr args (new InputParams);
  

static void readLine (ifstream& ifile, string& line)
{
    char buff[2048];
    fill_n (buff, 2048, '\0');
    ifile.getline (buff, 2048);
    line = buff;
}

static void getSIFTParameters (istringstream& istr)
{
  string keyword;
  string val;

  SIFTKeypointParams& par = *((SIFTKeypointParams*) args->keypoint_par.get ());
  par.keypoint_type = "sift";


  istr >> keyword >> val;
  if (keyword.compare ("par_min_scale") == 0) {
    par.min_scale = atof (val.c_str ());
    cout << "\tSIFT min scale = " << par.min_scale << endl;
  }
  else {
    throw std::runtime_error ("SIFT min scale not provided ..");
  }

  istr >> keyword >> val;
  if (keyword.compare ("par_n_octaves") == 0) {
    par.n_octaves = atoi (val.c_str ());
    cout << "\tSIFT No. of octaves = " << par.n_octaves << endl;
  }
  else {
    throw std::runtime_error ("SIFT No. of octaves not provided ..");
  }

  istr >> keyword >> val;
  if (keyword.compare ("par_n_scales_per_octave") == 0) {
    par.n_scales_per_octave = atoi (val.c_str ());
    cout << "\tSIFT No. of scales per octave = " << par.n_scales_per_octave << endl;
  }
  else {
    throw std::runtime_error ("SIFT No. of scales per octave not provided ..");
  }

  istr >> keyword >> val;
  if (keyword.compare ("par_k_search") == 0) {
    par.k_search = atoi (val.c_str ());
    cout << "\tSIFT K Search parameter = " << par.k_search << endl;
  }
  else {
    throw std::runtime_error ("SIFT K Search parameter not provided ..");
  }
}

static void getPrincipalCurvatureParameters (istringstream& istr)
{
  string keyword;
  string val;

  PrincipalCurvatureParams& par = *((PrincipalCurvatureParams*) args->keypoint_par.get ());
  par.keypoint_type = "principalcurvatures";

  istr >> keyword >> val;
  if (keyword.compare ("par_k_search") == 0) {
    par.k_search = atoi (val.c_str ());
    cout << "\tPrincipalCurvatures K Search parameter = " << par.k_search << endl;
  }
  else {
    throw std::runtime_error ("PrincipalCurvatures K Search parameter not provided ..");
  }
}

static void getCurvatureLocalMaximaParameters (istringstream& istr)
{
  string keyword;
  string val;

  CurvatureLocalMaximaParams& par = *((CurvatureLocalMaximaParams*) args->keypoint_par.get ());
  par.keypoint_type = "curvaturelocalmaxima";

  istr >> keyword >> val;
  if (keyword.compare ("par_k_search") == 0) {
    par.k_search = atoi (val.c_str ());
    cout << "\tCurvatureLocalMaxima K Search parameter = " << par.k_search << endl;
  }
  else {
    throw std::runtime_error ("CurvatureLocalMaxima K Search parameter not provided ..");
  }
}

static void getBoundaryPointsParams (istringstream& istr)
{
  string keyword;
  string val;

  BoundaryPointsParams& par = *((BoundaryPointsParams*) args->keypoint_par.get ());
  par.keypoint_type = "boundarypoints";

  istr >> keyword >> val;
  if (keyword.compare ("par_k_search") == 0) {
    par.k_search = atoi (val.c_str ());
    cout << "\tBoundaryPoints K Search parameter = " << par.k_search << endl;
  }
  else {
    throw std::runtime_error ("BoundaryPoints K Search parameter not provided ..");
  }

  istr >> keyword >> val;
  if (keyword.compare ("border_radius") == 0) {
    par.border_radius = atof (val.c_str ());
    cout << "\tBoundaryPoints border_radius parameter = " << par.border_radius << endl;
  }
  else {
    throw std::runtime_error ("BoundaryPoints border radius parameter not provided ..");
  }
}

static void getCurvatureParameters (istringstream& istr)
{
  string keyword;
  string val;

  CurvatureKeypointParams& par = *((CurvatureKeypointParams*) args->keypoint_par.get ());
  par.keypoint_type = "curvature";

  istr >> keyword >> val;
  if (keyword.compare ("par_k_search") == 0) {
    par.k_search = atoi (val.c_str ());
    cout << "\tCurvature K Search parameter = " << par.k_search << endl;
  }
  else {
    throw std::runtime_error ("Curvature K Search parameter not provided ..");
  }
  

  istr >> keyword >> val;
  if (keyword.compare ("par_num_points") == 0) {
    par.num_points = atoi (val.c_str ());
    cout << "\tCurvature num points parameter = " << par.num_points << endl;
  }
  else {
    throw std::runtime_error ("Curvature Num Points parameter not provided ..");
  }

}

static void getISSParameters (istringstream& istr)
{
  string keyword;
  string val;

  ISSKeypointParams& par = *((ISSKeypointParams*) args->keypoint_par.get ());
  par.keypoint_type = "iss";
  istr >> keyword >> val;
  if (keyword.compare ("par_mod_res") == 0) {
    par.model_resolution = atof (val.c_str ());
    cout << "\tISS model resolution = " << par.model_resolution << endl;
  }
  else {
    throw std::runtime_error ("ISS model resolution not provided ..");
  }

  istr >> keyword >> val;
  if (keyword.compare ("par_21_ratio") == 0) {
    par.ratio_21 = atof (val.c_str ());
    cout << "\tISS 21 ratio = " << par.ratio_21 << endl;
  }
  else {
    throw std::runtime_error ("ISS 21 ratio not provided ..");
  }


  istr >> keyword >> val;
  if (keyword.compare ("par_32_ratio") == 0) {
    par.ratio_32 = atof (val.c_str ());
    cout << "\tISS 32 ratio = " << par.ratio_32 << endl;
  }
  else {
    throw std::runtime_error ("ISS 32 ratio not provided ..");
  }

  istr >> keyword >> val;
  if (keyword.compare ("par_min_neighbours") == 0) {
    par.min_neighbours = atof (val.c_str ());
    cout << "\tISS neighbours = " << par.min_neighbours << endl;
  }
  else {
    throw std::runtime_error ("ISS min neighbours not provided ..");
  }
}

InputParamsPtr getInputParams ()
{
  return args;
}

int loadConfigFile (const char* filename)
{
  cout << endl;

  ifstream ifile (filename);
  if (!ifile) {
    cout << "Cannot load config file .. " << filename << " ..\n\n";
    return -1;
  }

  while (ifile) {

    string line;
    readLine (ifile, line);

    if ( (line.length () == 0) or (line[0] == '#')) {
      continue;
    }

    istringstream istr (line);
    string keyword;
    string val;

    istr >> keyword >> val;

    //cout << "keyword = " << keyword << " val = " << val << endl;

    if (keyword.compare ("source") == 0 ) {
      cout << "Source cloud = " << val << endl;
      args->sourcefile = val;
      
      readLine (ifile, line);
      //cout << "LINE :: " << line << endl;
      istringstream istr1 (line);

      keyword = "", val = "";
      istr1 >> keyword >> val;
      //cout << "keyword = " << keyword << " val = " << val << endl;
      if (keyword.compare ("type") == 0) {
        cout << "Type = " << val << "\n";
        args->filetype1 = val;
      }
      else {
        cout << "File type not entered for source cloud..\n";
        return -1;
      }
      continue;
    }

    if (keyword.compare ("target") == 0 ) {
      cout << "Target cloud = " << val << endl;
      args->targetfile = val;

      readLine (ifile, line);

      istringstream istr1 (line);

      keyword = "", val = "";
      istr1 >> keyword >> val;
      //cout << "keyword = " << keyword << " val = " << val << endl;
      if (keyword.compare ("type") == 0) {
        cout << "Type = " << val << "\n";
        args->filetype2 = val;
      }
      else {
        cout << "File type not entered for target cloud..\n";
        return -1;
      }
      continue;
    }

    if (keyword.compare ("congruency") == 0 ) {
      cout << "Congruency = " << val << endl;
      args->congruency = val;
      continue;
    }

    if (keyword.compare ("numquads") == 0 ) {
      cout << "Number of quads = " << val << endl;
      args->num_quads = atoi (val.c_str ());
      continue;
    }

    if (keyword.compare ("sampling") == 0 ) {

      args->sampling_type = val;

      if (args->sampling_type.compare ("random") == 0) {
        istr >> keyword >> val;
        if (keyword.compare ("ratio1") == 0) {
          args->random_sampling_ratio1 = atof (val.c_str ());
          cout << "\tRatio 1 = " << args->random_sampling_ratio1 << endl;
        }
        else {
          throw std::runtime_error ("Ratio1 parameter not provided ..");
        }

        istr >> keyword >> val;
        if (keyword.compare ("ratio2") == 0) {
          args->random_sampling_ratio2 = atof (val.c_str ());
          cout << "\tRatio 2 = " << args->random_sampling_ratio2 << endl;
        }
        else {
          throw std::runtime_error ("Ratio2 parameter not provided ..");
        }
      }

      else if (args->sampling_type.compare ("randomonwindows") == 0) {
        istr >> keyword >> val;
        if (keyword.compare ("ratio") == 0) {
          args->random_sampling_ratio1 = atof (val.c_str ());
          cout << "\tRatio  = " << args->random_sampling_ratio1 << endl;
        }
        else {
          throw std::runtime_error ("Ratio parameter not provided ..");
        }

        istr >> keyword >> val;
        if (keyword.compare ("windowsize") == 0) {
          args->windowsize = atof (val.c_str ());
          cout << "\tWindow Size  = " << args->windowsize << endl;
        }
        else {
          throw std::runtime_error ("Window size not provided ..");
        }
      }

      else if (args->sampling_type.compare ("keypoints") == 0) {
        istr >> keyword >> val;
        if (keyword.compare ("type") == 0) {
          args->keypoint_type = val;
          cout << "Keypoint type = " << args->keypoint_type << endl;

          if (args->keypoint_type.compare ("iss") == 0) {
            args->keypoint_par.reset (new ISSKeypointParams);
            getISSParameters (istr);
          }
          else if (args->keypoint_type.compare ("sift") == 0) {
            args->keypoint_par.reset (new SIFTKeypointParams);
            getSIFTParameters (istr);
          }
          else if (args->keypoint_type.compare ("curvature") == 0) {
            args->keypoint_par.reset (new CurvatureKeypointParams);
            getCurvatureParameters (istr);
          }
          else if (args->keypoint_type.compare ("curvaturelocalmaxima") == 0) {
            args->keypoint_par.reset (new CurvatureLocalMaximaParams);
            getCurvatureLocalMaximaParameters (istr);
          }
          else if (args->keypoint_type.compare ("principalcurvatures") == 0) {
            args->keypoint_par.reset (new PrincipalCurvatureParams);
            getPrincipalCurvatureParameters (istr);
          }
          else if (args->keypoint_type.compare ("boundarypoints") == 0) {
            args->keypoint_par.reset (new BoundaryPointsParams);
            getBoundaryPointsParams (istr);
          }
        }
        else {
          throw std::runtime_error ("Sampling type not provided ..");
        }
      }

      else if (args->sampling_type.compare ("keypointsandregionsaround") == 0) {
        istr >> keyword >> val;
        if (keyword.compare ("type") == 0) {
          args->keypoint_type = val;
          cout << "Keypoint type = " << args->keypoint_type << endl;

          if (args->keypoint_type.compare ("iss") == 0) {
            args->keypoint_par.reset (new ISSKeypointParams);
            getISSParameters (istr);
          }
          else if (args->keypoint_type.compare ("sift") == 0) {
            args->keypoint_par.reset (new SIFTKeypointParams);
            getSIFTParameters (istr);
          }
          else if (args->keypoint_type.compare ("curvature") == 0) {
            args->keypoint_par.reset (new CurvatureKeypointParams);
            getCurvatureParameters (istr);
          }

          istr >> keyword >> val;

          if (keyword.compare ("region") == 0) {
            args->region_around_radius = atof (val.c_str ());
            cout << "Region around radius = " << args->region_around_radius << endl;
          }
          else {
            throw std::runtime_error ("Region size not mentioned ..");
          }

        }
        else {
          throw std::runtime_error ("Sampling type not provided ..");
        }
      }

      else if (args->sampling_type.compare ("keypointsandrandom") == 0) {
        istr >> keyword >> val;
        if (keyword.compare ("type") == 0) {
          args->keypoint_type = val;
          cout << "Keypoint type = " << args->keypoint_type << endl;

          if (args->keypoint_type.compare ("iss") == 0) {
            args->keypoint_par.reset (new ISSKeypointParams);
            getISSParameters (istr);
          }
          else if (args->keypoint_type.compare ("sift") == 0) {
            args->keypoint_par.reset (new SIFTKeypointParams);
            getSIFTParameters (istr);
          }
          else if (args->keypoint_type.compare ("curvature") == 0) {
            args->keypoint_par.reset (new CurvatureKeypointParams);
            getCurvatureParameters (istr);
          }

          istr >> keyword >> val;

          if (keyword.compare ("ratio") == 0) {
            args->random_sampling_ratio1 = atof (val.c_str ());
            cout << "Random sampling ratio = " << args->random_sampling_ratio1 << endl;
          }
          else {
            throw std::runtime_error ("Random sampling ratio not provided ..");
          }
        }
        else {
          throw std::runtime_error ("Sampling type not provided ..");
        }
      }
      else {
        string msg = string ("Unknown sampling type :: ") + args->sampling_type;
        throw std::runtime_error (msg);
      }

      continue;

    }

    if (keyword.compare ("errorballdiameter") == 0 ) {
      cout << "Error ball diameter = " << val << endl;
      args->D = atof (val.c_str ());
      continue;
    }

    if (keyword.compare ("abcd_mindist") == 0 ) {
      cout << "ABCD min dist = " << val << endl;
      args->abcd_mindist = atof (val.c_str ());
      continue;
    }

    if (keyword.compare ("corr_max_range") == 0 ) {
      cout << "Correspondence max range = " << val << endl;
      args->corr_max_range = atof (val.c_str ());
      continue;
    }

    if (keyword.compare ("offset") == 0 ) {
      cout << "Offset = " << val << endl;
      args->offset = atof (val.c_str ());
      continue;
    }

    if (keyword.compare ("vis_sampling") == 0 ) {
      cout << "Visualization Sampling = " << val << endl;
      args->vis_num_points = atoi (val.c_str ());
      continue;
    }

    if (keyword.compare ("vis_sphereradius") == 0 ) {
      cout << "Visualization Sphere radius = " << val << endl;
      args->sphere_radius = atof (val.c_str ());
      continue;
    }
  }
  cout << endl << endl;
}




}
