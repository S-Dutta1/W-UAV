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

#include "cs251_base.hpp"
#include <cstdio>
#include <vector>
#include <iostream>
using namespace std;
using namespace cs251;

int fnear_detected=0,ffar_detected=0,fnear_isFalse=0,both_onffar=0,monitoring=0,blimp_reached=0,all_done=0,w1=0;
base_sim_t::base_sim_t()
{
	b2Vec2 gravity;
	gravity.Set(0.0f, 0.0f);
	m_world = new b2World(gravity);

	m_text_line = 30;

	m_point_count = 0;

	m_world->SetDebugDraw(&m_debug_draw);
	
	m_step_count = 0;

	b2BodyDef body_def;
	m_ground_body = m_world->CreateBody(&body_def);

	memset(&m_max_profile, 0, sizeof(b2Profile));
	memset(&m_total_profile, 0, sizeof(b2Profile));
}

base_sim_t::~base_sim_t()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
	m_world = NULL;
}

void base_sim_t::pre_solve(b2Contact* contact, const b2Manifold* oldManifold)
{
  const b2Manifold* manifold = contact->GetManifold();
  
  if (manifold->pointCount == 0)
    {
      return;
    }
  
  b2Fixture* fixtureA = contact->GetFixtureA();
  b2Fixture* fixtureB = contact->GetFixtureB();
  
  b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
  b2GetPointStates(state1, state2, oldManifold, manifold);
  
  b2WorldManifold world_manifold;
  contact->GetWorldManifold(&world_manifold);
  
  for (int32 i = 0; i < manifold->pointCount && m_point_count < k_max_contact_points; ++i)
    {
      contact_point_t* cp = m_points + m_point_count;
      cp->fixtureA = fixtureA;
      cp->fixtureB = fixtureB;
      cp->position = world_manifold.points[i];
      cp->normal = world_manifold.normal;
      cp->state = state2[i];
      ++m_point_count;
    }
}

void base_sim_t::draw_title(int x, int y, const char *string)
{
    m_debug_draw.DrawString(x, y, string);
}

  

void base_sim_t::step(settings_t* settings)
{
  float32 time_step = settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

  if (settings->pause)
    {
      if (settings->single_step)
	{
	  settings->single_step = 0;
	}
      else
	{
	  time_step = 0.0f;
	}
      
      m_debug_draw.DrawString(5, m_text_line, "****PAUSED****");
      m_text_line += 15;
    }
  

  uint32 flags = 0;
  flags += settings->draw_shapes			* b2Draw::e_shapeBit;
  flags += settings->draw_joints			* b2Draw::e_jointBit;
  flags += settings->draw_AABBs			* b2Draw::e_aabbBit;
  flags += settings->draw_pairs			* b2Draw::e_pairBit;
  flags += settings->draw_COMs				* b2Draw::e_centerOfMassBit;
  m_debug_draw.SetFlags(flags);
  
  m_world->SetWarmStarting(settings->enable_warm_starting > 0);
  m_world->SetContinuousPhysics(settings->enable_continuous > 0);
  m_world->SetSubStepping(settings->enable_sub_stepping > 0);
  
  m_point_count = 0;
  
  m_world->Step(time_step, settings->velocity_iterations, settings->position_iterations);

  /***********************CODE HERE*****************************/
  
  b2Body* bdptr = m_world->GetBodyList();
  vector<b2Body*> bodylist;

  while(bdptr->GetNext()){
    bodylist.push_back(bdptr);
    bdptr=bdptr->GetNext();
  }
  int n=bodylist.size();
  

  b2Vec2 vel;
  vel.x=5.0f;vel.y=0;

  b2Vec2 ballvel;
  ballvel.x=0.0f;ballvel.y=-5.0f;

  b2Vec2 zero_vel;
  zero_vel.x=0;zero_vel.y=0;

  b2Body* sballc=bodylist[0];
  b2Body* blimp1=bodylist[n-2];//blimp
  b2Body* cop_upper=bodylist[n-3];//upper copter
  b2Body* cop_lower=bodylist[n-4];//lower copter
  //b2Body* ffar=bodylist[n-5];//fire1-farther by lower coptr
  //b2Body* fnear=bodylist[n-6];//fire2- nearer one

  b2Vec2 gvel = cop_lower->GetLinearVelocity();
  b2Vec2 bmp_vel=blimp1->GetLinearVelocity();
  //cout<<gvel.x<<endl;
  if(bmp_vel.x==0 && blimp_reached==0)
  {
    blimp1->SetLinearVelocity(vel);
  }

  if(blimp1->GetWorldCenter().x>17)
  {
    blimp_reached=1;
    blimp1->SetLinearVelocity(zero_vel);
  }

  if(blimp_reached==1)
    w1++;

  if(gvel.x==0 && fnear_detected==0 && ffar_detected==0 && blimp_reached==1 && w1==300)
  {
    cop_upper->SetLinearVelocity(vel);
    cop_lower->SetLinearVelocity(vel);
  }


  if(cop_upper->GetWorldCenter().x>8.5f && fnear_detected==0)
  {
    //cout<<"here"<<endl;
    fnear_detected=1;
    cop_upper->SetLinearVelocity(zero_vel);
    
    b2CircleShape scircle;
    scircle.m_radius = 0.5f;
        
    b2FixtureDef sball;
    sball.shape = &scircle;
    sball.density = 1.0f;
    sball.friction = 0.0f;
    sball.restitution = 0.7f;
    b2BodyDef ballbds;
    ballbds.type = b2_dynamicBody;

    ballbds.position.Set(8.8f, 19.0f);
    sballc = m_world->CreateBody(&ballbds);
    sballc->CreateFixture(&sball);

    sballc->SetLinearVelocity(ballvel);
  }

  //cout<<b2Distance(cop_lower->GetWorldCenter(),f1->GetWorldCenter())<<endl;
  if(cop_lower->GetWorldCenter().x>25 && ffar_detected ==0)
  {
    //cout<<"here"<<endl;
    ffar_detected=1;
    cop_lower->SetLinearVelocity(zero_vel);
  }
  
  if(ffar_detected==1)
    fnear_isFalse++;//time for false fire detection

  cout<<(cop_upper->GetLinearVelocity()).x<<endl;
  if((sballc->GetLinearVelocity()).y>0 )
  {
    //delete fnear;
    sballc->SetLinearVelocity(zero_vel);
    cop_upper->SetLinearVelocity(vel);
    
  }

  if(cop_upper->GetWorldCenter().x>25.f && ffar_detected ==1)
  {
    both_onffar=1;
    cop_upper->SetLinearVelocity(zero_vel);
  }  

  if(both_onffar==1)
    monitoring++;  

  if(monitoring==500)
  {
    cop_upper->SetLinearVelocity(-vel);
    cop_lower->SetLinearVelocity(-vel);
    blimp1->SetLinearVelocity(-vel);
    all_done=1;
  }

  if(all_done==1 && cop_lower->GetWorldCenter().x<-30)
  {
    //cout<<"hey"<<endl;
    cop_upper->SetLinearVelocity(zero_vel);
    cop_lower->SetLinearVelocity(zero_vel);
  }

  if(all_done==1 && blimp1->GetWorldCenter().x<-30)
  {
    blimp1->SetLinearVelocity(zero_vel);
  }




  /****************************************************/
  m_world->DrawDebugData();
  
  if (time_step > 0.0f)
    {
      ++m_step_count;
    }
  
  if (settings->draw_stats)
    {
      int32 body_count = m_world->GetBodyCount();
      int32 contact_count = m_world->GetContactCount();
      int32 joint_count = m_world->GetJointCount();
      m_debug_draw.DrawString(5, m_text_line, "bodies/contacts/joints = %d/%d/%d", body_count, contact_count, joint_count);
      m_text_line += 15;
      
      int32 proxy_count = m_world->GetProxyCount();
      int32 height = m_world->GetTreeHeight();
      int32 balance = m_world->GetTreeBalance();
      float32 quality = m_world->GetTreeQuality();
      m_debug_draw.DrawString(5, m_text_line, "proxies/height/balance/quality = %d/%d/%d/%g", proxy_count, height, balance, quality);
      m_text_line += 15;
    }
  
  // Track maximum profile times
  {
    const b2Profile& p = m_world->GetProfile();
    m_max_profile.step = b2Max(m_max_profile.step, p.step);
    m_max_profile.collide = b2Max(m_max_profile.collide, p.collide);
    m_max_profile.solve = b2Max(m_max_profile.solve, p.solve);
    m_max_profile.solveInit = b2Max(m_max_profile.solveInit, p.solveInit);
    m_max_profile.solveVelocity = b2Max(m_max_profile.solveVelocity, p.solveVelocity);
    m_max_profile.solvePosition = b2Max(m_max_profile.solvePosition, p.solvePosition);
    m_max_profile.solveTOI = b2Max(m_max_profile.solveTOI, p.solveTOI);
    m_max_profile.broadphase = b2Max(m_max_profile.broadphase, p.broadphase);
    
    m_total_profile.step += p.step;
    m_total_profile.collide += p.collide;
    m_total_profile.solve += p.solve;
    m_total_profile.solveInit += p.solveInit;
    m_total_profile.solveVelocity += p.solveVelocity;
    m_total_profile.solvePosition += p.solvePosition;
    m_total_profile.solveTOI += p.solveTOI;
    m_total_profile.broadphase += p.broadphase;
  }
  
  if (settings->draw_profile)
    {
      const b2Profile& p = m_world->GetProfile();
      
      b2Profile ave_profile;
      memset(&ave_profile, 0, sizeof(b2Profile));
      if (m_step_count > 0)
	{
	  float32 scale = 1.0f / m_step_count;
	  ave_profile.step = scale * m_total_profile.step;
	  ave_profile.collide = scale * m_total_profile.collide;
	  ave_profile.solve = scale * m_total_profile.solve;
	  ave_profile.solveInit = scale * m_total_profile.solveInit;
	  ave_profile.solveVelocity = scale * m_total_profile.solveVelocity;
	  ave_profile.solvePosition = scale * m_total_profile.solvePosition;
	  ave_profile.solveTOI = scale * m_total_profile.solveTOI;
	  ave_profile.broadphase = scale * m_total_profile.broadphase;
	}
      
      m_debug_draw.DrawString(5, m_text_line, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, ave_profile.step, m_max_profile.step);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, ave_profile.collide, m_max_profile.collide);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, ave_profile.solve, m_max_profile.solve);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, ave_profile.solveInit, m_max_profile.solveInit);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, ave_profile.solveVelocity, m_max_profile.solveVelocity);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, ave_profile.solvePosition, m_max_profile.solvePosition);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, ave_profile.solveTOI, m_max_profile.solveTOI);
      m_text_line += 15;
      m_debug_draw.DrawString(5, m_text_line, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, ave_profile.broadphase, m_max_profile.broadphase);
      m_text_line += 15;
    }
    
  if (settings->draw_contact_points)
    {
      //const float32 k_impulseScale = 0.1f;
      const float32 k_axis_scale = 0.3f;
      
      for (int32 i = 0; i < m_point_count; ++i)
	{
	  contact_point_t* point = m_points + i;
	  
	  if (point->state == b2_addState)
	    {
	      // Add
	      m_debug_draw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
	    }
	  else if (point->state == b2_persistState)
	    {
	      // Persist
	      m_debug_draw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
	    }
	  
	  if (settings->draw_contact_normals == 1)
	    {
	      b2Vec2 p1 = point->position;
	      b2Vec2 p2 = p1 + k_axis_scale * point->normal;
	      m_debug_draw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
	    }
	  else if (settings->draw_contact_forces == 1)
	    {
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->normalForce * point->normal;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
	    }
	  
	  if (settings->draw_friction_forces == 1)
	    {
	      //b2Vec2 tangent = b2Cross(point->normal, 1.0f);
	      //b2Vec2 p1 = point->position;
	      //b2Vec2 p2 = p1 + k_forceScale * point->tangentForce * tangent;
	      //DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
	    }
	}
    }
}
