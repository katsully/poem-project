#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include "cinder/params/Params.h"
#include "kat_decision_tree.h"
#include "cinder/osc/Osc.h"

using namespace ci;
using namespace ci::app;
using namespace std;

#define USE_UDP 1

#if USE_UDP
using Receiver = osc::ReceiverUdp;
using protocol = asio::ip::udp;
#else
using Receiver = osc::ReceiverTcp;
using protocol = asio::ip::tcp;
#endif

const uint16_t localPort = 10001;

class poemProjectApp : public App {
  public:
	  poemProjectApp();
	void setup() override;
	void update() override;
	void draw() override;

private:
	Kinect2::DeviceRef mDevice;
	Kinect2::BodyFrame mBodyFrame;
	ci::Channel8uRef mChannelBodyIndex;
	ci::Channel16uRef mChannelDepth;

	params::InterfaceGlRef mParams;
	bool mDrawBackground = true;
	bool mDrawSkeleton = false;
	bool mFullScreen = false;
	bool mShowParams = true;

	bool mStill = false;
	int mStillCounter = 0;

	float distShoulder, distElbow, distKnee;

	vec3 prevShoulder = vec3(0);
	vec3 prevElbow = vec3(0);
	vec3 prevKnee = vec3(0);

	vec3 currShoulder, currElbow, currKnee;

	vector<double> features;
	bool mStartRecording = true;
	vec3 spineFirst;
	int pose;
	int currPose, prevPose, poseHold = 0;

	Receiver mReceiver;
	bool mRecording = false;

	vector<string> poem{ "I like to think",
		"(right now, please!)",
	"of a cybernetic forest",
		"filled with pines and electronics",
		"where deer stroll peacefully" };
	// For drawing the poem
	Font mFont;
	gl::TextureFontRef mTextureFont;
	vector<string> newPoem;
};

poemProjectApp::poemProjectApp() : mReceiver (localPort)
{
	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame frame) {
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame frame) {
		mChannelBodyIndex = frame.getChannel();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame frame) {
		mChannelDepth = frame.getChannel();
	});
}

void poemProjectApp::setup()
{
	setFullScreen(mFullScreen);

	//vector<double> test{ 0,1,2,3,4,5,-0.20717 ,7,8,9,-0.557962 , 0.267425,12,13,0.267062 ,15,0.0573292,17,18,19,20,21 };
	//console() << "results: " << kat_decision_tree(test) << endl;

	// create a parameter interface and name it
	mParams = params::InterfaceGl::create(getWindow(), "App Parameters", toPixels(ivec2(200, 400)));

	// setup parameters
	mParams->addParam("Draw Background", &mDrawBackground);
	mParams->addParam("Draw Skeleton", &mDrawSkeleton);

	mParams->addSeparator();

	mParams->addParam("Full Screen", &mFullScreen).updateFn([this] { setFullScreen(mFullScreen); });
	mParams->addParam("Show Params", &mShowParams).key("p");

	mReceiver.bind();
	mReceiver.listen(
		[](asio::error_code error, protocol::endpoint endpoint) -> bool {
		if (error) {
			return false;
		}
		else
			return true;
	});
	mReceiver.setListener("/1/toggle1", [&](const osc::Message &message) {
		if (message[0].flt() == 1.0) {
			console() << "skfjslkdjfls" << endl;
			mDrawSkeleton = true;
		}
		else if(message[0].flt() == 0.0){
			mDrawSkeleton = false;
		}
	});

	// load font
	mFont = Font("Avenir", 96);
	mTextureFont = gl::TextureFont::create(mFont);
}

void poemProjectApp::update()
{
}

void poemProjectApp::draw()
{
	const gl::ScopedViewport scopedViewport(ivec2(0), getWindowSize());
	const gl::ScopedMatrices scopedMatrices;
	const gl::ScopedBlendAlpha scopedBlendAlpha;
	gl::setMatricesWindow(getWindowSize());
	gl::clear();
	if (mStill) {
		gl::color(Color(1, 0, 0));
	}
	else {
		gl::color(Color(1, 1, 1));
	}
	gl::disableDepthRead();
	gl::disableDepthWrite();

	if (mChannelDepth && mDrawBackground) {
		gl::enable(GL_TEXTURE_2D);
		gl::TextureRef tex = gl::Texture::create(*Kinect2::channel16To8(mChannelDepth));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowBounds()));
	}

	if (mChannelBodyIndex && mDrawSkeleton) {
			gl::enable(GL_TEXTURE_2D);
			gl::color(ColorAf(Colorf::white(), 0.15f));
			gl::TextureRef tex = gl::Texture::create(*Kinect2::colorizeBodyIndex(mChannelBodyIndex));
			gl::draw(tex, tex->getBounds(), Rectf(getWindowBounds()));

			gl::pushMatrices();
			gl::scale(vec2(getWindowSize()) / vec2(mChannelBodyIndex->getSize()));
			gl::disable(GL_TEXTURE_2D);
			for (const Kinect2::Body &body : mBodyFrame.getBodies()) {
				if (body.isTracked()) {
					gl::color(ColorAf::white());

					auto map = body.getJointMap();

					// get spine of first frame
					if (mStartRecording == true) {
						spineFirst = map.at(JointType_SpineBase).getPosition();
						mStartRecording = false;
						console() << "this only happens once" << endl;
					}

					// determine if person is moving or standing still
					currShoulder = map.at(JointType_ShoulderLeft).getPosition();
					currElbow = map.at(JointType_ElbowRight).getPosition();
					currKnee = map.at(JointType_KneeRight).getPosition();

					distShoulder = sqrt(math<float>::pow(currShoulder.x - prevShoulder.x, 2) + math<float>::pow(currShoulder.y - prevShoulder.y, 2) + math<float>::pow(currShoulder.z - prevShoulder.z, 2));
					distElbow = sqrt(math<float>::pow(currElbow.x - prevElbow.x, 2) + math<float>::pow(currElbow.y - prevElbow.y, 2) + math<float>::pow(currElbow.z - prevElbow.z, 2));
					distKnee = sqrt(math<float>::pow(currKnee.x - prevKnee.x, 2) + math<float>::pow(currKnee.y - prevKnee.y, 2) + math<float>::pow(currKnee.z - prevKnee.z, 2));

					if (distShoulder <= 0.008 && distElbow <= 0.04 && distKnee <= 0.06) {
						mStillCounter += 1;
					}
					else {
						mStillCounter = 0;
					}

					if (mStillCounter > 15) {
						mStill = true;
					}
					else {
						mStill = false;
					}

					prevShoulder = currShoulder;
					prevElbow = currElbow;
					prevKnee = currKnee;

					if (mStill) {

						// normalize
						vec3 normSpineBase = vec3(map.at(JointType_SpineBase).getPosition().x - spineFirst.x, map.at(JointType_SpineBase).getPosition().y - spineFirst.y, map.at(JointType_SpineBase).getPosition().z - spineFirst.z);
						vec3 normHead = vec3(map.at(JointType_Head).getPosition().x - spineFirst.x, map.at(JointType_Head).getPosition().y - spineFirst.y, map.at(JointType_Head).getPosition().z - spineFirst.z);
						vec3 normSpineMid = vec3(map.at(JointType_SpineMid).getPosition().x - spineFirst.x, map.at(JointType_SpineMid).getPosition().y - spineFirst.y, map.at(JointType_SpineMid).getPosition().z - spineFirst.z);
						vec3 normHipLeft = vec3(map.at(JointType_HipLeft).getPosition().x - spineFirst.x, map.at(JointType_HipLeft).getPosition().y - spineFirst.y, map.at(JointType_HipLeft).getPosition().z - spineFirst.z);
						vec3 normHipRight = vec3(map.at(JointType_HipRight).getPosition().x - spineFirst.x, map.at(JointType_HipRight).getPosition().y - spineFirst.y, map.at(JointType_HipRight).getPosition().z - spineFirst.z);
						vec3 normSpineShoulder = vec3(map.at(JointType_SpineShoulder).getPosition().x - spineFirst.x, map.at(JointType_SpineShoulder).getPosition().y - spineFirst.y, map.at(JointType_SpineShoulder).getPosition().z - spineFirst.z);
						vec3 normWristLeft = vec3(map.at(JointType_WristLeft).getPosition().x - spineFirst.x, map.at(JointType_WristLeft).getPosition().y - spineFirst.y, map.at(JointType_WristLeft).getPosition().z - spineFirst.z);
						vec3 normWristRight = vec3(map.at(JointType_WristRight).getPosition().x - spineFirst.x, map.at(JointType_WristRight).getPosition().y - spineFirst.y, map.at(JointType_WristRight).getPosition().z - spineFirst.z);
						vec3 normElbowLeft = vec3(map.at(JointType_ElbowLeft).getPosition().x - spineFirst.x, map.at(JointType_ElbowLeft).getPosition().y - spineFirst.y, map.at(JointType_ElbowLeft).getPosition().z - spineFirst.z);
						vec3 normElbowRight = vec3(map.at(JointType_ElbowRight).getPosition().x - spineFirst.x, map.at(JointType_ElbowRight).getPosition().y - spineFirst.y, map.at(JointType_ElbowRight).getPosition().z - spineFirst.z);
						vec3 normShoulderRight = vec3(map.at(JointType_ShoulderRight).getPosition().x - spineFirst.x, map.at(JointType_ShoulderRight).getPosition().y - spineFirst.y, map.at(JointType_ShoulderRight).getPosition().z - spineFirst.z);
						vec3 normShoulderLeft = vec3(map.at(JointType_ShoulderLeft).getPosition().x - spineFirst.x, map.at(JointType_ShoulderLeft).getPosition().y - spineFirst.y, map.at(JointType_ShoulderLeft).getPosition().z - spineFirst.z);
						vec3 normKneeLeft = vec3(map.at(JointType_KneeLeft).getPosition().x - spineFirst.x, map.at(JointType_KneeLeft).getPosition().y - spineFirst.y, map.at(JointType_KneeLeft).getPosition().z - spineFirst.z);
						vec3 normKneeRight = vec3(map.at(JointType_KneeRight).getPosition().x - spineFirst.x, map.at(JointType_KneeRight).getPosition().y - spineFirst.y, map.at(JointType_KneeRight).getPosition().z - spineFirst.z);
						vec3 normAnkleRight = vec3(map.at(JointType_AnkleRight).getPosition().x - spineFirst.x, map.at(JointType_AnkleRight).getPosition().y - spineFirst.y, map.at(JointType_AnkleRight).getPosition().z - spineFirst.z);
						vec3 normAnkleLeft = vec3(map.at(JointType_AnkleLeft).getPosition().x - spineFirst.x, map.at(JointType_AnkleLeft).getPosition().y - spineFirst.y, map.at(JointType_AnkleLeft).getPosition().z - spineFirst.z);


						// feature one - Distance between Spine base and Head
						// feature[0]
						features.push_back(sqrt(math<float>::pow(normSpineBase.x - normHead.x, 2) + math<float>::pow(normSpineBase.y - normHead.y, 2) + math<float>::pow(normSpineBase.z - normHead.z, 2)));

						// feature two - Mean position between Spine mid, Hip left and Hip right
						vec3 feature2 = vec3((normHipLeft.x + normHipRight.x + normSpineMid.x) / 3, (normHipLeft.y + normHipRight.y + normSpineMid.y) / 3, (normHipLeft.z + normHipRight.z + normSpineMid.z) / 3);
						// feature[1]
						features.push_back(feature2.x);
						// feature[2]
						features.push_back(feature2.y);
						// feature[3]
						features.push_back(feature2.z);

						// feature three - XY position of the torso joints
						// feature[4]
						features.push_back(normSpineShoulder.x);
						// feature[5]
						features.push_back(normSpineShoulder.y);

						// feature[6]
						features.push_back(normSpineMid.x);
						// feature[7]
						features.push_back(normSpineMid.y);

						// feature[8]
						features.push_back(normSpineBase.x);
						// feature[9]
						features.push_back(normSpineBase.y);

						// feature four - avg between wrist and elbow (left and right) and knee and ankle (left and right)
						vector<vec3> feature4;
						// features[10,11,12]
						feature4.push_back(vec3((normWristLeft.x + normElbowLeft.x + normShoulderLeft.x) / 3, (normWristLeft.y + normElbowLeft.y + normShoulderLeft.y) / 3, (normWristLeft.z + normElbowLeft.z + normShoulderLeft.z) / 3));
						vec3 temp = vec3((normWristLeft.x + normElbowLeft.x + normShoulderLeft.x) / 3, (normWristLeft.y + normElbowLeft.y + normShoulderLeft.y) / 3, (normWristLeft.z + normElbowLeft.z + normShoulderLeft.z) / 3);
						// features[13,14,15]
						feature4.push_back(vec3((normWristRight.x + normElbowRight.x + normShoulderRight.x) / 3, (normWristRight.y + normElbowRight.y + normShoulderRight.y) / 3, (normWristRight.z + normElbowRight.z + normShoulderRight.z) / 3));
						feature4.push_back(vec3((normHipRight.x + normKneeRight.x + normAnkleRight.x) / 3, (normHipRight.y + normKneeRight.y + normAnkleRight.y) / 3, (normHipRight.z + normKneeRight.z + normAnkleRight.z) / 3));
						feature4.push_back(vec3((normHipLeft.x + normKneeLeft.x + normAnkleLeft.x) / 3, (normHipLeft.y + normKneeLeft.y + normAnkleLeft.y) / 3, (normHipLeft.z + normKneeLeft.z + normAnkleLeft.z) / 3));

						for (int i = 0; i < feature4.size(); i++) {
							features.push_back(feature4[i].x);
							features.push_back(feature4[i].y);
							features.push_back(feature4[i].z);
						}
						currPose = kat_decision_tree(features);

						if (currPose == prevPose) {
							poseHold += 1;
							if (poseHold > 20) {
								pose = currPose;
								console() << "it happened! " << pose << endl;
								console() << "features: " << "14: " << features[14] << " 11: " << features[11] << " 20: " << features[20] << endl;
								poseHold = 0;
								mStillCounter = 0;
							}
						}
						else {
							poseHold = 0;
						}

						prevPose = currPose;
						feature4.clear();
						features.clear();
					}

					for (const auto& joint : body.getJointMap()) {
						if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
							if (mStill) {
								gl::color(Color(1, 0, 0));
							}
							else {
								gl::color(Color(0, 0, 1));
							}
							vec2 pos(mDevice->mapCameraToDepth(joint.second.getPosition()));
							gl::drawSolidCircle(pos, 5.0f, 32);
							vec2 parent(mDevice->mapCameraToDepth(
								body.getJointMap().at(joint.second.getParentJoint()).getPosition()
							));
							gl::drawLine(pos, parent);
						}
					}
				}
			}
			gl::popMatrices();
		}

		if (newPoem.size() > 11) {
			newPoem.clear();
		}

		if (pose == 0) {
			newPoem.push_back(poem[0]);
			pose = -10;
		}
		else if (pose == 1) {
			newPoem.push_back(poem[1]);
			pose = -10;
	}
		else if (pose == 2) {
			newPoem.push_back(poem[2]);
			pose = -10;
		}
		else if (pose == 3) {
			newPoem.push_back(poem[3]);
			pose = -10;
		}
		else if (pose == 4) {
			newPoem.push_back(poem[4]);
			pose = -10;
		}

		for (int i = 0; i < newPoem.size(); i++) {
			gl::color(Color::white());
			mTextureFont->drawString(newPoem[i], vec2(50, i*100+50));
	}

	

	// draw parameters interface
	if (mShowParams) {
		mParams->draw();
	}
}

CINDER_APP( poemProjectApp, RendererGl )
