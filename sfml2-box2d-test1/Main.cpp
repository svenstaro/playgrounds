/* Main.cpp -- SFML + Box2D test project entry point.  

  Copyright (C) 2010 Allen Jordan

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Allen Jordan allen.jordan@gmail.com
*/

#include "GlobalInclude.h"
#include "GameController.h"

//main entry point to the program
int main(int argc, char** argv)
{
	B2_NOT_USED(argc);
	B2_NOT_USED(argv);

	//redirect cerr to a file
	ofstream errorFile("ErrorLog.txt");
	cerr.rdbuf(errorFile.rdbuf());

	int result = EXIT_SUCCESS;

	try
	{
		//create the game controller class
		GameController gameController;
		//run the game controller
		result = gameController.OnExecute();
	}
	catch (const exception &ex)
	{
		//output any errors to the std::cerr stream
		cerr << ex.what() << endl;
		result = EXIT_FAILURE;
	}

	return result;
}
