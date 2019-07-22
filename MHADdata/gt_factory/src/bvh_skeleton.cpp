/*
  BVH SKELETON CLASS
  ==================
  Modified from the code found at
  https://www.gamedev.net/articles/programming/general-and-gameplay-programming/bvh-file-loading-and-displaying-r3295/
  by Edin Mujagic.

 */

#define Xposition 0x01
#define Yposition 0x02
#define Zposition 0x04

#define Zrotation 0x10
#define Xrotation 0x20
#define Yrotation 0x40

typedef struct
{
  float x, y, z;
} OFFSET;

typedef struct JOINT JOINT;

struct JOINT
{
  const char* name = NULL;        // joint name
  JOINT* parent = NULL;           // joint parent
  OFFSET offset;                  // offset data
  unsigned int num_channels = 0;  // num of channels joint has
  short* channel_order = NULL;    // ordered lits of channels
  std::vector children;           // joints children
  glm::mat4 matrix;               // local transformation matrix (premultiplied with parents')
  unsigned int channel_start = 0; // idx of joint's channel data in motion array
};

typedef struct
{
  JOINT* rootJoint;
  int num_channels;
} HIERARCHY;

typedef struct
{
  unsigned int num_frames;               // number of frames
  unsigned int num_motion_channels = 0;  // number of motion channels
  float* data = NULL;                    // motion float data array
  unsigned* joint_channel_offsets;       // number of channels from beginning of hierarchy for i-th joint
} MOTION;
	   
 
public class BVH_Skeleton
{
  JOINT* loadJoint(std::istream& stream, JOINT* parent = NULL); // TODO
  void loadHierarchy(std::istream& stream);
  void loadMotion(std::istream& stream);

public:
  BVH_Skeleton();
  ~BVH_Skeleton();

  // loading
  void load(const std::string& filename);

  /** Loads motion data from a frame into local matrices **/
  void moveTo(unsigned frame);

  const JOINT* getRootJoint() const { return rootJoint; }
  unsigned getNumFrames() const {return motionData.num_frames; }

private:
  JOINT* rootJoint;
  MOTION motionData;
};

void BVH_Skeleton::load(const std::string& filename)
{
  std::fstream file;
  file.open(filename.c_str(), std::ios_base::in);

  if( file.is_open() ){
    std::string line;

    while( file.good() ){
      
      file >> line;
      if( trim(line) == "HIERARCHY" )
	loadHierarchy(file);
      break;
    }

    file.close();
  }
}

void BVH_Skeleton::loadHierarchy(std::istream& stream)
{
  std:: string tmp;

  while( stream.good() ){
    stream >> tmp;

    if( trim(tmp) == "ROOT")
      rootJoint = loadJoint(stream);
    else if( trim(tmp) == "MOTION")
      loadMotion(stream);
  }
}


JOINT* BVH_Skeleton::loadJoint(std::istream& stream, JOINT* parent)
{
  JOINT* joint = new JOINT;
  joint->parent = parent;

  // load joint name
  std::string* name = new std::string;
  stream >> *name;
  joint->name = name->c_str();

  std::string tmp;

  // setting local matrix to identity matrix
  joint->matrix = glm::mat4(1.0);

  static int _channel_start = 0;
  unsigned channel_order_idx = 0;

  while( stream.good() ){
    stream >> tmp;
    tmp = trim(tmp);

    // loading channels
    char c = tmp.at(0);

    if( c == 'X' || c == 'Y' || c == 'Z' ) {

      if( tmp == "Xposition" ) {
	joint->channels_order[channel_order_idx++] = Xposition;
      }
      if( tmp == "Yposition" ) {
	joint->channels_order[channel_order_idx++] = Yposition;
      }
      if( tmp == "Zposition" ) {
	joint->channels_order[channel_order_idx++] = Zposition;
      }

      if( tmp == "Xrotation" ) {
	joint->channels_order[channel_order_idx++] = Xrotation;
      }
      if( tmp == "Yrotation" ) {
	joint->channels_order[channel_order_idx++] = Yrotation;
      }
      if( tmp == "Zrotation" ) {
	joint->channels_order[channel_order_idx++] = Zrotation;
      }
    }

    if( tmp == "OFFSET" ){
      // reading an offset value
      stream >> joint->offset.x
	     >> joint->offset.y
	     >> joint->offset.z;
    }
    else if( tmp == "CHANNELS" ){
      // loading number of channels
      stream >> joint->num_channels;

      // adding to motion data
      motionData.num_motion_channels += joint->num_channels;

      // increasing static counter of channel index starting motion section
      joint->channel_start = _channel_start;
      _channel_start += joint->num_channels;

      // creating array for channel order specification
      joint->channels_order = new short[joint->num_channels];
    }
    else if( tmp == "JOINT" ){
      // loading child joint and setting this as parent
      JOINT* tmp_joint = loadJoint(stream, joint);

      tmp_joint->parent = joint;
      joint->children.push_back(tmp_joint);
    }
    else if( tmp == "End" ){
      // loading End Site joint
      stream >> tmp >> tmp; // Site {

      JOINT* tmp_joint = new JOINT;

      tmp_joint->parent = joint;
      tmp_joint->num_channels = 0;
      tmp_joint->name = "EndSite";
      joint->children.push_back(tmp_joint);

      stream >> tmp;
      if( tmp == "OFFSET" )
	stream >> tmp_joint->offset.x
	       >> tmp_joint->offset.y
	       >> tmp_joint->offset.z;

      stream >> tmp;
    }
    else if( tmp == "}" )
      return joint;
    
    
  } // wend

}


void BVH_Skeleton::loadMotion(std::istream& stream)
{
  std::string tmp;

  while( stream.good() ){
    stream >> tmp;

    if( trim(tmp) == "Frames:" ){
      // loading frame number
      stream >> motionData.num_frames;
    }
    else if( trim(tmp) == "Frame" ){
      // loading frame time
      float frame_time;
      stream >> tmp >> frame_time;

      int num_frames = motionData.num_frames;
      int num_channels = motionData.num_motion_channels;

      // creating motion data array
      motionData = new float[num_frames * num_channels];

      // foreach frame and store floats
      for( int frame = 0; frame < num_frames; frame++ ){
	for( int channel = 0; channel < num_channels; channel++ ){

	  // reading float
	  float x;
	  std::stringstream ss;
	  stream >> tmp;
	  ss << tmp;
	  ss >> x;

	  // calculating index for storage
	  int index = frame * num_channels + channel;
	  motionData.data[index] = x;
	  
	} // for channel end
      } // for frame end
    } // if "Frame" end

  } // wend
}


/**
   Calculates JOINT's local transformation matrix for
   specified frame starting index
*/
static void moveJoint(JOINT* joint, MOTION* motionData, int frame_starts_index)
{
  // we'll need the index of motion data's array with start of this specific joint
  int start_index = frame_starts_index + joint->channel_start;

  // translate identity matrix to this joint's offset parameters
  joint->matrix = glm::translate(glm::mat4(1.0),
				 glm::vec3(joint->offset.x,
					   joint->offset.y,
					   joint->offset.z));

  // here we transform joint;s local matrix with each specified channel's values, which
  // are read from motion data
  for( int i = 0; i < joint->num_channels; i++ ){
    // channel alias
    const short& channel = joint->channels_order;

    // extract value from motion data
    float value = motionData->data[start_index + i];

    if( channel & Xposition ){
      joint->matrix = glm::translate(joint->matrix, glm::vec3(value, 0, 0));
    }
    if( channel & Yposition ){
      joint->matrix = glm::translate(joint->matrix, glm::vec3(0, value, 0));
    }
    if( channel & Zposition ){
      joint->matrix = glm::translate(joint->matrix, glm::vec3(0, 0, value));
    }

    if( channel & Xrotation ){
      joint->matrix = glm::rotate(joint->matrix, glm::vec3(1, 0, 0));
    }
    if( channel & Yrotation ){
      joint->matrix = glm::rotate(joint->matrix, glm::vec3(0, 1, 0));
    }
    if( channel & Zrotation ){
      joint->matrix = glm::rotate(joint->matrix, glm::vec3(0, 0, 1));
    }
  }

  // then we apply parent's local transformation matrix to this joint's LTM (local tr.mat.)
  if( joint->parent != NULL )
    joint->matrix = joint->parent->matrix * joint->matrix;

  // when we have calculated parent's matrix, do the same to all children
  for( auto& child : joint->children )
    moveJoint(child, motionData, frame_starts_index);
}

void BVH_Skeleton::moveTo(unsigned frame)
{
  // we calculate motion data's array start index for a frame
  unsigned start_index = frame * motionData.num_motion_channels;

  // recursively transform skeleton
  moveJoint(rootJoint, &motionData, start_index);
}
