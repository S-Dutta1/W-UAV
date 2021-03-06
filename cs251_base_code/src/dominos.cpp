/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  
  dominos_t::dominos_t()
  {
    //Ground
    /*! \var b1 
     * \brief pointer to the body ground 
     */ 
    b2Body* b1;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }

    //  ////circular blimp///////////
    // {
    //   b2Body* blimp;
      
    //   b2CircleShape circle;
    //   circle.m_radius = 5.0;
  
    //   b2FixtureDef bmp;
    //   bmp.shape = &circle;
    //   bmp.density = 50.0f;
    //   bmp.friction = 0.0f;
    //   bmp.restitution = 0.0f;
    //   b2BodyDef ballbd;
    //   ballbd.type = b2_staticBody;
    //   ballbd.position.Set(-30.0f, 30.0f);
    //   blimp = m_world->CreateBody(&ballbd);
    //   blimp->CreateFixture(&bmp);
    // }    


    ///////ellipse blimp/////////////////////
    {
      b2Body* tblp;

      b2Vec2 vertices[24];

      vertices[0].Set(0.2f, 0);
      vertices[1].Set(2, 0.3f);
      vertices[2].Set(4, 0.9f);
      vertices[3].Set(6, 1.6f);
      vertices[4].Set(8,2.5f);
      vertices[5].Set(10,3.8f);

      vertices[6].Set(10,4.2f);
      vertices[7].Set(8,5.5f);
      vertices[8].Set(6,6.4f);
      vertices[9].Set(4,7.1f);
      vertices[10].Set(2,7.7f);
      vertices[11].Set(0.2f,8);

      vertices[12].Set(-0.2f, 8);
      vertices[13].Set(-2, 7.7f);
      vertices[14].Set(-4, 7.1f);
      vertices[15].Set(-6, 6.4f);
      vertices[16].Set(-8,5.5f);
      vertices[17].Set(-10,4.2f);

      vertices[18].Set(-10,3.8f);
      vertices[19].Set(-8,2.5f);
      vertices[20].Set(-6,1.6f);
      vertices[21].Set(-4,0.9f);
      vertices[22].Set(-2,0.3f);
      vertices[23].Set(-0.2f,0);
      
      
      
      int count = 24;
      b2PolygonShape polygon;
      polygon.Set(vertices, count);


      b2FixtureDef bmp1;
      bmp1.shape = &polygon;
      bmp1.density = 50.0f;
      bmp1.friction = 0.0f;
      bmp1.restitution = 0.0f;
      b2BodyDef ballbd1;
      ballbd1.type = b2_dynamicBody;
      ballbd1.position.Set(-30.0f, 30.0f);
      tblp = m_world->CreateBody(&ballbd1);
      tblp->CreateFixture(&bmp1);
      ///////////////////////blimp///////////////////////


      //////////////copters/////////////////////
    
      b2Body* cop1;b2Body* cop2;b2Body* fire1;b2Body* fire2;

      b2Vec2 vertices1[4];
      vertices1[0].Set(0, 0);
      vertices1[1].Set(3, 1);
      vertices1[2].Set(0, 2);
      vertices1[3].Set(-3, 1);

      b2PolygonShape polygon1;
      polygon1.Set(vertices1, 4);  

      b2FixtureDef helis;
      helis.shape = &polygon1;
      helis.density = 50.0f;
      helis.friction = 0.0f;
      helis.restitution = 0.0f;
      b2BodyDef ballbd2;
      ballbd2.type = b2_dynamicBody;
      ballbd2.position.Set(-30.0f, 23.0f);


      cop1 = m_world->CreateBody(&ballbd2);
      cop1->CreateFixture(&helis);    

      ballbd2.position.Set(-30.0f, 20.0f);
      cop2 = m_world->CreateBody(&ballbd2);
      cop2->CreateFixture(&helis);

      ////FIRE///////////////////////////////////////////////
      b2Vec2 verticesfire[4];
      verticesfire[0].Set(0, 0);
      verticesfire[1].Set(2, 0);
      verticesfire[2].Set(2, 2);
      verticesfire[3].Set(0, 2);

      b2PolygonShape polygonfire;
      polygonfire.Set(verticesfire, 4);  

      b2FixtureDef fff;
      fff.shape = &polygonfire;
      fff.density = 50.0f;
      fff.friction = 0.0f;
      fff.restitution = 0.0f;
      b2BodyDef firesquare;
      firesquare.type = b2_staticBody;

      firesquare.position.Set(8.8f, 0.0f);
      fire1 = m_world->CreateBody(&firesquare);
      fire1->CreateFixture(&fff);

      // firesquare.position.Set(8.0f, 0.0f);
      // fire2 = m_world->CreateBody(&firesquare);
      // fire2->CreateFixture(&fff);
      {
      	{
        b2PolygonShape shape;
        shape.SetAsBox(0.01f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 2000.0f;
        fd.friction = 1.0f;

        b2PolygonShape shape2;
        shape2.SetAsBox(0.01f, 0.8f);

        b2FixtureDef fd2;
        fd2.shape = &shape2;
        fd2.density = 2000.0f;
        fd2.friction = 0.1f;

        b2PolygonShape shape3;
        shape3.SetAsBox(0.001f, 1.7f);

        b2FixtureDef fd3;
        fd3.shape = &shape3;
        fd3.density = 2000.0f;
        fd3.friction = 0.1f;

        b2PolygonShape shape4;
        shape4.SetAsBox(0.001f, 2.2f);

        b2FixtureDef fd4;
        fd4.shape = &shape4;
        fd4.density = 2000.0f;
        fd4.friction = 0.1f;

        b2PolygonShape shape5;
        shape5.SetAsBox(0.001f, 2.8f);

        b2FixtureDef fd5;
        fd5.shape = &shape5;
        fd5.density = 2000.0f;
        fd5.friction = 0.1f;


        for (int i = 0; i < 20; ++i)
        {

            b2BodyDef bd;
            bd.type = b2_staticBody;
            bd.position.Set(25.3f + 0.25f * i, 2.0f );
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
            bd.position.Set(8.3f + 0.25f * i, 2.0f );
            // body = m_world->CreateBody(&bd);
            // body->CreateFixture(&fd);

        }
        for (int i = 0; i < 5; ++i)
        {

            b2BodyDef bd;
            bd.type = b2_staticBody;
            bd.position.Set(25.3f + 1.f * i, 2.0f );
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd2);
            bd.position.Set(8.3f + 1.f * i, 2.0f );
            // body = m_world->CreateBody(&bd);
            // body->CreateFixture(&fd2);

        }
        for (int i = 0; i < 10; ++i)
        {

            b2BodyDef bd;
            bd.type = b2_staticBody;
            bd.position.Set(25.4f + 0.5f * i, 2.0f );
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd3);
            bd.position.Set(8.4f + 0.5f * i, 2.0f );
            // body = m_world->CreateBody(&bd);
            // body->CreateFixture(&fd3);

        }
        for (int i = 0; i < 5; ++i)
        {

            b2BodyDef bd;
            bd.type = b2_staticBody;
            bd.position.Set(26.f + 1.f * i, 2.0f );
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd4);
            bd.position.Set(9.f + 1.f * i, 2.0f );
            // body = m_world->CreateBody(&bd);
            // body->CreateFixture(&fd4);

        }
        for (int i = 0; i < 3; ++i)
        {

            b2BodyDef bd;
            bd.type = b2_staticBody;
            bd.position.Set(26.f + 1.5f * i, 2.0f );
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd5);
            bd.position.Set(9.f + 1.5f * i, 2.0f );
            // body = m_world->CreateBody(&bd);
            // body->CreateFixture(&fd5);

        }
    }
      }
    }
    

       
    
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
