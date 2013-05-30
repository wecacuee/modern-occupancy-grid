// file: cvmat_serialization.h
//http://cheind.wordpress.com/2011/12/06/serialization-of-cvmat-objects-using-boost/
//
#include <cstdio>

#include <map>
#include <fstream>
#include <string>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/assign/list_of.hpp> // for 'map_list_of()'

#include <opencv2/opencv.hpp>

#include <cvmat_serialization.h>

 
////////////////////////////
// An unnecessary incomplete attempt to allow cross cpp/python binary
// serialization of cv::Mat
////////////////////////////
// std::map<int, std::string> DEPTH2FIELD =
//   boost::assign::map_list_of(1, "X")(2, "X Y")(3, "X Y Z")(4, "X Y Z NORMAL");
// const std::map<int, std::string> TYPE2PCDTYPE =
//   boost::assign::map_list_of(1, "F")(2, "F");
// 
// int save(cv::Mat img, std::ofstream ofs) {
//     ofs << "# .PCD v.7 - Point Cloud Data file format\n";
//     ofs << "VERSION .7\n";
//     std::string fields = DEPTH2FIELD[ img.depth() ];
//     ofs << "FIELDS " << fields << std::endl;
// 
//     std::stringstream counts("");
//     std::stringstream types("");
//     std::stringstream size("");
//     for (int i = 0; i < img.depth(); i++) {
//         size << img.elemSize1() << " ";
//         types << "F" << " ";
//         counts << "1 ";
//     }
//     ofs << "SIZE " << size.str() << std::endl;
//     ofs << "TYPE " << types.str() << std::endl;
//     ofs << "COUNT " << counts.str() << std::endl;
//     ofs << "WIDTH " << img.size[1] << std::endl;
//     ofs << "HEIGHT " << img.size[0] << std::endl;
//     ofs << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
//     ofs << "POINTS " << img.total() << std::endl;
//     ofs << "DATA ";
//     ofs.write((const char *)img.data, img.total() * img.elemSize());
// }
 
BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
  namespace serialization {
 
    /** Serialization support for cv::Mat */
    template <typename Archive>
    void save(Archive & ar, const cv::Mat& m, const unsigned int version)
    {
      size_t elem_size = m.elemSize();
      size_t elem_type = m.type();
 
      ar & m.cols;
      ar & m.rows;
      ar & elem_size;
      ar & elem_type;
 
      const size_t data_size = m.cols * m.rows * elem_size;
      ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
    /** Serialization support for cv::Mat */
    template <typename Archive>
    void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
    {
      int cols, rows;
      size_t elem_size, elem_type;
 
      ar & cols;
      ar & rows;
      ar & elem_size;
      ar & elem_type;
 
      m.create(rows, cols, elem_type);
 
      size_t data_size = m.cols * m.rows * elem_size;
      ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
  }
}
void saveMat(cv::Mat& m, std::string filename) {
    std::ofstream ofs(filename.c_str());
	boost::archive::binary_oarchive oa(ofs);
	//boost::archive::text_oarchive oa(ofs);
	oa << m;
}

void loadMat(cv::Mat& m, std::string filename) {
	std::ifstream ifs(filename.c_str());
	boost::archive::binary_iarchive ia(ifs);
	//boost::archive::text_iarchive ia(ifs);
	ia >> m;
}
