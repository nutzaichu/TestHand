/*
* 
* Copyright (c) 2013, Wieden+Kennedy
* Stephen Schieberl, Michael Latzoni
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"
#include "cinder/Font.h"
#include "Kinect2.h"


using namespace ci;
using namespace ci::app;
using namespace std;
using namespace Kinect2;
using namespace gl;

class BasicApp : public ci::app::AppBasic 
{
public:
	void						draw();
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						setup();
	void						update();
	Kinect2::DeviceRef			mDevice;
	Kinect2::Frame				mFrame;
	Kinect2::Body				mBody;
	vector<Body>				mBodies;
	float						mFrameRate;
	bool						mFullScreen;
	map<JointType, Body::Joint>	jointMap;
	ci::params::InterfaceGlRef	mParams;
	Texture						mTexture;
	Body::Joint					handRight;
	Body::Joint					handLeft;
	float						handRightx;
	float						handRighty;
	float						handLeftx;
	float						handLefty;

	float						handRightx2;
	float						handRighty2;
	float						handLeftx2;
	float						handLefty2;

	Vec2f						handRightxy;
	Vec2f						handLeftxy;

	Vec2f						handRightxyPrint;
	Vec2f						handLeftxyPrint;

	Vec2i						handRightxyScreen;
	Vec2i						handLeftxyScreen;

	Vec2f						handRightxyScreen2;
	Vec2f						handLeftxyScreen2;

	Vec3f						handRightxy2;
	Vec3f						handLeftxy2;
	ICoordinateMapper*			mCoorMapper;
private:

};

void BasicApp::prepareSettings( Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 1280, 720 ).title( "Basic App" ) );
	settings->setFrameRate( 60.0f );
}

void BasicApp::setup()
{	
	gl::enable( GL_TEXTURE_2D );
	
	mFrameRate	= 0.0f;
	mFullScreen	= false;

	mDevice = Kinect2::Device::create();
	mDevice->start( Kinect2::DeviceOptions().enableBodyIndex().enableBody() );
	
}

void BasicApp::update()
{
	mFrameRate = getAverageFps();
	
	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}

	if ( mDevice && mDevice->getFrame().getTimeStamp() > mFrame.getTimeStamp() ) {
		mFrame = mDevice->getFrame();
	}

	mBodies = mFrame.getBodies();
	console() << mBodies.size() << endl;
	for (int i = 0; i < mBodies.size(); i++){
		mBody = mBodies[i];
		//if (mBody.isTracked()){
		jointMap = mBody.getJointMap();
		handRight = jointMap[JointType_HandRight];
		handLeft = jointMap[JointType_HandLeft];
		handRightx = (handRight.getPosition()[0] * 1000) + 640;
		handRighty = (handRight.getPosition()[1] * -600) + 360;
		handLeftx = (handLeft.getPosition()[0] * 1000) + 640;
		handLefty = (handLeft.getPosition()[1] * -600) + 360;
		handRightxy = Vec2f(handRightx, handRighty);
		handLeftxy = Vec2f(handLeftx, handLefty);
	}

	mCoorMapper = mDevice->getCoordinateMapper();

	handRightxyScreen = mapBodyCoordToColor(handRight.getPosition(), mCoorMapper);
	handLeftxyScreen = mapBodyCoordToColor(handLeft.getPosition(), mCoorMapper);
	

	handRightxyScreen2 = Vec2f(handRightxyScreen);
	handLeftxyScreen2 = Vec2f(handRightxyScreen);




}

void BasicApp::draw()
{
	gl::setViewport(getWindowBounds());
	gl::clear();
	gl::setMatricesWindow(getWindowSize());
	//gl::enableAlphaBlending();

	if (mFrame.getColor()) {
		gl::TextureRef tex = gl::Texture::create(mFrame.getColor());
		gl::draw(tex, tex->getBounds(), Rectf(Vec2f::zero(), Vec2f(1280,720)));
	}
	/*
	if (mFrame.getDepth()) {
		gl::TextureRef tex = gl::Texture::create(Kinect2::channel16To8(mFrame.getDepth()));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowCenter().x, 0.0f, (float)getWindowWidth(), getWindowCenter().y));
	}
	if (mFrame.getInfrared()) {
		gl::TextureRef tex = gl::Texture::create(mFrame.getInfrared());
		gl::draw(tex, tex->getBounds(), Rectf(0.0f, getWindowCenter().y, getWindowCenter().x, (float)getWindowHeight()));
	}
	if (mFrame.getBodyIndex()) {
		gl::TextureRef tex = gl::Texture::create(Kinect2::colorizeBodyIndex(mFrame.getBodyIndex()));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowCenter(), Vec2f(getWindowSize())));
	}
	*/
	//if (mFrame.getBodyIndex()){ 
		drawSolidCircle(handRightxy, 50, 0);
		drawSolidCircle(handLeftxy, 50, 0);
	//}
	//mParams->draw();
}
CINDER_APP_BASIC( BasicApp, RendererGl )
	