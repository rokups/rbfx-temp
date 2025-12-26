//
// Copyright (c) 2025 the rbfx project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include "CsgShowcase.h"

#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Input/FreeFlyController.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>

#include <Urho3D/DebugNew.h>

using namespace Urho3D;

CsgShowcase::CsgShowcase(Context* context)
    : Sample(context)
{
}

void CsgShowcase::Start()
{
    Sample::Start();

    CreateScene();
    CreateInstructions();
    SetupViewport();

    SetMouseMode(MM_RELATIVE);
    SetMouseVisible(false);
}

void CsgShowcase::CreateScene()
{
    auto* cache = GetSubsystem<ResourceCache>();

    scene_ = MakeShared<Scene>(context_);
    if (!scene_->LoadFile("Scenes/CsgShowcase.scene"))
    {
        URHO3D_LOGERROR("Failed to load scene: Scenes/CsgShowcase.scene");
        return;
    }

    cameraNode_ = scene_->CreateChild("Camera");
    cameraNode_->CreateComponent<Camera>();
    cameraNode_->SetPosition(Vector3(0.0f, 4.0f, -10.0f));
    cameraNode_->CreateComponent<FreeFlyController>();
}

void CsgShowcase::CreateInstructions()
{
    auto* cache = GetSubsystem<ResourceCache>();

    instructionText_ = GetUIRoot()->CreateChild<Text>();
    instructionText_->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    instructionText_->SetHorizontalAlignment(HA_CENTER);
    instructionText_->SetVerticalAlignment(VA_TOP);
    instructionText_->SetPosition(0, 10);

    instructionText_->SetText("CSG Showcase\nWASD + mouse to fly (FreeFlyController)");
}

void CsgShowcase::SetupViewport()
{
    auto* renderer = GetSubsystem<Renderer>();

    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    SetViewport(0, viewport);
}
