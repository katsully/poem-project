#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Kinect2.h"
#include "cinder/params/Params.h"

using namespace ci;
using namespace ci::app;
using namespace std;

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
	bool mDrawBackground = false;
	bool mDrawSkeleton = false;
	bool mFullScreen = false;
	bool mShowParams = false;

	bool mStill = false;
	int mStillCounter = 0;

	float distShoulder, distElbow, distKnee;

	vec3 prevShoulder = vec3(0);
	vec3 prevElbow = vec3(0);
	vec3 prevKnee = vec3(0);

	vec3 currShoulder, currElbow, currKnee;
};

poemProjectApp::poemProjectApp() 
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

	// create a parameter interface and name it
	mParams = params::InterfaceGl::create(getWindow(), "App Parameters", toPixels(ivec2(200, 400)));

	// setup parameters
	mParams->addParam("Draw Background", &mDrawBackground);
	mParams->addParam("Draw Skeleton", &mDrawSkeleton);
	
	mParams->addSeparator();

	mParams->addParam("Full Screen", &mFullScreen).updateFn([this] { setFullScreen(mFullScreen); });
	mParams->addParam("Show Params", &mShowParams).key("p");
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

				// determine if person is moving or standing still
				currShoulder = body.getJointMap().at(JointType_ShoulderLeft).getPosition();
				currElbow = body.getJointMap().at(JointType_ElbowRight).getPosition();
				currKnee = body.getJointMap().at(JointType_KneeRight).getPosition();

				distShoulder = sqrt(math<float>::pow(currShoulder.x - prevShoulder.x, 2) + math<float>::pow(currShoulder.y - prevShoulder.y, 2) + math<float>::pow(currShoulder.z - prevShoulder.z, 2));
				distElbow = sqrt(math<float>::pow(currElbow.x - prevElbow.x, 2) + math<float>::pow(currElbow.y - prevElbow.y, 2) + math<float>::pow(currElbow.z - prevElbow.z, 2));
				distKnee = sqrt(math<float>::pow(currKnee.x - prevKnee.x, 2) + math<float>::pow(currKnee.y - prevKnee.y, 2) + math<float>::pow(currKnee.z - prevKnee.z, 2));

				if (distShoulder <= 0.004 && distElbow <= 0.008 && distKnee <= 0.02) {
					mStillCounter += 1;
				}
				else {
					mStillCounter = 0;
				}

				if (mStillCounter > 20) {
					mStill = true;
				}
				else {
					mStill = false;
				}

				prevShoulder = currShoulder;
				prevElbow = currElbow;
				prevKnee = currKnee;

				for (const auto& joint : body.getJointMap()) {
					if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
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

	// draw parameters interface
	if (mShowParams) {
		mParams->draw();
	}
}

CINDER_APP( poemProjectApp, RendererGl )
