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
#include "cinder/Rand.h"

using namespace ci;
using namespace ci::app;
using namespace std;
using namespace Kinect2;
using namespace gl;

struct Hand
{
public:
	Hand(Vec2i screenPos, HandState state);
	Vec2i						screenPos;
	HandState					state;
};
class BasicApp : public ci::app::AppBasic 
{
public:
	void						draw();
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						setup();
	void						update();
	void						drawHand();
	void						drawFruit();
	Kinect2::DeviceRef			mDevice;
	Kinect2::Frame				mFrame;
	Kinect2::Body				mBody;
	vector<Body>				mBodies;
	float						mFrameRate;
	bool						mFullScreen;
	map<JointType, Body::Joint>	jointMap;
	Texture						mTexture;
	Body::Joint					handRight;
	Body::Joint					handLeft;
	ci::Rand					rand;
	vector<Vec2f>				fruit;
	vector<Hand>				right;
	vector<Hand>				left;
	ICoordinateMapper*			mCoorMapper;
	float						w;
	float						h;
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
	
	for (int i = 0; i < 1; i++){ 
		w = rand.nextFloat(300.0f, 1000.0f);
		h = rand.nextFloat(200.0f, 700.0f);
		fruit.push_back(Vec2f(w,h));
	}
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
	///////// GET BODIES /////////////
	mBodies = mFrame.getBodies();

	///////// GET HAND FROM EACH BODY ///////////
	for (int i = 0; i < mBodies.size(); i++){
		mBody = mBodies[i];
		if (mBody.isTracked()){
			jointMap = mBody.getJointMap();
			handRight = jointMap[JointType_HandRight]; 
			handLeft = jointMap[JointType_HandLeft];
			Vec2i handRightScreen = mapBodyCoordToColor(handRight.getPosition(), mCoorMapper);
			Vec2i handLeftScreen = mapBodyCoordToColor(handLeft.getPosition(), mCoorMapper);
			if (mFrame.getColor() && mDevice)
			{
				handRightScreen *= (Vec2f(getWindowSize()) / Vec2f(mFrame.getColor().getSize()));
				handLeftScreen *= (Vec2f(getWindowSize()) / Vec2f(mFrame.getColor().getSize()));
			}

			///////// CHECK IF TOUCH CIRCLE /////////////
			for (int j = 0; j < fruit.size(); j++){
				if ((abs(handRightScreen.x - fruit[j].x) < 10.0f && abs(handRightScreen.y - fruit[j].y < 10.0f)) || (abs(handLeftScreen.x - fruit[j].x) < 10.0f && abs(handLeftScreen.y - fruit[j].y < 10.0f))) {
					w = rand.nextFloat(300.0f, 900.0f);
					h = rand.nextFloat(200.0f, 600.0f);
					fruit[j] = Vec2f(w, h);
				}
			}

			///////// KEEP FOR DRAWING ////////////
			right.push_back(Hand(handRightScreen, mBody.getRightHandState()));
			left.push_back(Hand(handLeftScreen, mBody.getLeftHandState()));

			
		}
	}
}

void BasicApp::draw()
{
	gl::setViewport(getWindowBounds());
	gl::clear(Colorf::black());
	gl::setMatricesWindow(getWindowSize());
	gl::enableAlphaBlending();
	gl::color(Colorf::white());
	if (mFrame.getColor()) {
		gl::TextureRef tex = gl::Texture::create(mFrame.getColor());
		gl::draw(tex, tex->getBounds(), Rectf(Vec2f::zero(), Vec2f(1280,720)));
	}
	drawHand();
	gl::color(Colorf::white());
	drawFruit();
}

void BasicApp::drawFruit()
{
	pushMatrices();
	for (int i = 0; i < fruit.size(); i++){
		drawSolidCircle(fruit[i], 60, 0);
	}
	popMatrices();
	
}
void BasicApp::drawHand()
{
	if (mFrame.getColor() && mDevice){
		pushMatrices();
		mCoorMapper = mDevice->getCoordinateMapper();
		for (int i = 0; i < right.size(); i++){
			if (right[i].state == HandState_Closed) {
				gl::color(255, 0, 0);
				drawStrokedCircle(right[i].screenPos, 20, 0);
			}
			else if (right[i].state == HandState_Open) {
				gl::color(255, 0, 0);
				drawStrokedCircle(right[i].screenPos, 20, 0);
			}
		}
		for (int i = 0; i < left.size(); i++){
			if (left[i].state == HandState_Closed) {
				gl::color(255, 0, 0);
				drawStrokedCircle(left[i].screenPos, 20, 0);
			}
			else if (left[i].state == HandState_Open) {
				gl::color(255, 0, 0);
				drawStrokedCircle(left[i].screenPos, 20, 0);
			}
		}
		popMatrices();
	}
}

CINDER_APP_BASIC( BasicApp, RendererGl )
	