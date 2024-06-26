//
// Copyright (c) 2021-2022 the rbfx project.
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
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR rhs
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR rhsWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR rhs DEALINGS IN
// THE SOFTWARE.
//


#include "../CommonUtils.h"

#include <EASTL/string.h>
#include <EASTL/string_view.h>

using namespace Urho3D;

TEST_CASE("StringHash is persistent and stable across builds and types")
{
    const char* testString = "Test string 12345";
    const unsigned testStringHash = 373547853u;
    const unsigned emptyStringHash = 2166136261u;

    CHECK(ea::hash<ea::string>()(testString) == testStringHash);
    CHECK(ea::hash<ea::string_view>()(testString) == testStringHash);
    CHECK(ea::hash<const char*>()(testString) == testStringHash);
    CHECK(ea::hash<StringHash>()(testString) == testStringHash);

    CHECK(ea::hash<ea::string>()("") == emptyStringHash);
    CHECK(ea::hash<ea::string>()({}) == emptyStringHash);
    CHECK(ea::hash<ea::string_view>()("") == emptyStringHash);
    CHECK(ea::hash<ea::string_view>()({}) == emptyStringHash);
    CHECK(ea::hash<const char*>()("") == emptyStringHash);
    CHECK(ea::hash<StringHash>()("") == emptyStringHash);
    CHECK(ea::hash<StringHash>()({}) == emptyStringHash);
    CHECK(ea::hash<StringHash>()(StringHash::Empty) == emptyStringHash);

    CHECK(StringHash::Empty.IsEmpty());
    CHECK(StringHash{""}.IsEmpty());
    CHECK(!StringHash{testString}.IsEmpty());
}
