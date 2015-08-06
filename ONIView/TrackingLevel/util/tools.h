#pragma once

#include <math.h>
#include <assert.h>

#define M_PI       3.14159265358979323846

namespace tools {

  float dist_2d( float x1, float y1, float x2, float y2 );

  float scale_to_range( float value, float src_min, float src_max, float dst_min, float dst_max );

  static inline float pow2(float x) {
    return x*x;
  }
  static inline float pow4(float x) {
    float xx = x*x;
    return xx * xx;
  }

  static inline float r2d(float r) {
    return r / ((float)M_PI) * 180.0f;
  }

  static inline float d2r(float d) {
    return d / 180.0f * ((float)M_PI);
  }

}  // namespace tools


//-----------------------------------------------------------------------------
struct ImageIndexer {

private: 
  int _rows, _cols;

public:
  inline int rows() const             {return _rows;}
  inline int cols() const             {return _cols;}
  inline int width() const            {return cols(); } 
  inline int height() const           {return rows(); } 
  int N;

  ImageIndexer( int rows=0, int cols=0 ) { setSize(rows,cols); }
  void setSize( int rows, int cols ) { _rows=rows; _cols=cols; N=rows*cols; }

  inline int xy2i(int x, int y) const {
    int i = x + (rows() - 1 - y) * cols();
    assert( i>=0 && i<N );
    return i;
  }

  inline void i2xy(int i, int &x, int &y) const {
    x = i % cols();
    y = rows() - 1 - (i / cols());
  }

  inline int rc2i(int r, int c) const {
    int i = c + r * cols();
    assert( i>=0 && i<N );
    return i;
  }

  inline void i2rc(int i, int &r, int &c) const {
    c = i % cols();
    r = i / cols();
  }


  bool is_border_point( int r, int c ) const {
    return (r>0 && r<_rows-1 && c>0 && c<_cols-1);
  }

  bool is_valid_cell( int r, int c ) const {
    return (r>=0 && r<=_rows-1 && c>=0 && c<=_cols-1);
  }

  bool is_valid_cell( int i ) const {
    return (i>=0 && i<N);
  }

  int sq_dist( int i, int j ) const {
    int r,c,r2,c2;
    i2rc(i,r,c);
    i2rc(j,r2,c2);
    //return abs(r-r2)+abs(c-c2);
    return (r-r2)*(r-r2) + (c-c2)*(c-c2);
  }

};




