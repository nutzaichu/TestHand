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
	Hand() : screenPos(0, 0), state(HandState_NotTracked){}
	Hand(Vec2i sp, HandState s){
		screenPos = sp;
		state = s;
	}
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
	vector<Vec2f>				fruitVel;
	vector<Hand>				right;
	vector<Hand>				left;
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
	
	for (int i = 0; i < 3; i++){ 
		float w = rand.nextFloat(200.0f, 1100.0f);
		float h = rand.nextFloat(100.0f, 600.0f);
		float vel = rand.nextFloat(5.0f, 15.0f);
		fruit.push_back(Vec2f(w,-10.0f));
		//fruitVel.push_back(Vec2f(0.0f, vel));
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
	right.resize(mBodies.size());
	left.resize(mBodies.size());
	for (int i = 0; i < mBodies.size(); i++){
		mBody = mBodies[i];
		if (mBody.isTracked()){
			jointMap = mBody.getJointMap();
			handRight = jointMap[JointType_HandRight]; 
			handLeft = jointMap[JointType_HandLeft];
			Vec2i handRightScreen = mapBodyCoordToColor(handRight.getPosition(), mCoorMapper);
			Vec2i handLeftScreen = mapBodyCoordToColor(handLeft.getPosition(), mCoorMapper);
			Vec2f handRightScreen2 = Vec2f(handRightScreen);
			Vec2f handLeftScreen2 = Vec2f(handLeftScreen);
			if (mFrame.getColor() && mDevice)
			{
				handRightScreen2 *= (Vec2f(getWindowSize()) / Vec2f(mFrame.getColor().getSize()));
				handLeftScreen2 *= (Vec2f(getWindowSize()) / Vec2f(mFrame.getColor().getSize()));
			}

			///////// CHECK IF TOUCH CIRCLE /////////////
			HandState rightState = mBody.getRightHandState();
			HandState leftState = mBody.getLeftHandState();
			
			for (int j = 0; j < fruit.size(); j++){
				float distanceRightX = abs(handRightScreen2.x - fruit[j].x);
				float distanceRightY = abs(handRightScreen2.y - fruit[j].y);
				float distanceLeftX = abs(handLeftScreen2.x - fruit[j].x);
				float distanceLeftY = abs(handLeftScreen2.y - fruit[j].y);
				if (((distanceRightX < 10.0f) && (distanceRightY < 10.0f)) || ((distanceLeftX < 10.0f) && (distanceLeftY < 10.0f))) {
					float w = rand.nextFloat(200.0f, 1100.0f);
					//h = rand.nextFloat(100.0f, 600.0f);
					fruit[j] = Vec2f(w, 0.0f);
				}
			}

			/////////// UPDATE POS ////////////////
			for (int j = 0; j < fruit.size(); j++){
				fruit[j] += fruitVel[j];
			}

			///////// KEEP FOR DRAWING ////////////
			right[i] = (Hand(handRightScreen2, rightState));
			left[i] = (Hand(handLeftScreen2, leftState));

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
	drawFruit();
	gl::color(Colorf::white());
	drawHand();
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

		for (int i = 0; i < mBodies.size(); i++){	
			if (right[i].state == HandState_Closed) {
				gl::color(255, 0 , 0);
				drawSolidCircle(right[i].screenPos, 20, 0);
			}
			else if (right[i].state == HandState_Open){
				gl::color(0, 255, 0);
				drawSolidCircle(right[i].screenPos, 20, 0);
			}
			else if (right[i].state == HandState_Lasso){
				gl::color(0, 0, 255);
				drawSolidCircle(right[i].screenPos, 20, 0);
			}
			
			if (left[i].state == HandState_Closed) {
				gl::color(255, 0, 0);
				drawSolidCircle(left[i].screenPos, 20, 0);
			}
			else if (left[i].state == HandState_Open){
				gl::color(0, 255, 0);
				drawSolidCircle(left[i].screenPos, 20, 0);
			}
			else if (left[i].state == HandState_Lasso){
				gl::color(0, 0, 255);
				drawSolidCircle(left[i].screenPos, 20, 0);
			}
		}
		popMatrices();
	}
}

CINDER_APP_BASIC( BasicApp, RendererGl )
	