#include "oooarkavr/math/Matrix.hpp"
#include "oooarkavr/math/Vector.hpp"
#include "oooarkavr/control/Controller.hpp"


namespace oooarkavr
{

class FeedbackController : public Controller
{
public:
    FeedbackController(int freq, Matrix<float> * feedbackGain, Vector<float> * state) : Controller(freq), state(state), feedbackGain(feedbackGain)
    {
        control.setSize(feedbackGain->getRows()+1);
    }
    virtual ~FeedbackController()
    {
    }
    void run()
    {
        control = (*feedbackGain)*(*state)*-1;
        control(1) = 0;
        //control.print(Serial,"Control Commanded: ");
        //Serial.println("control running");
    }

    //Class members
    Vector<float> * state;
    Matrix<float> * feedbackGain;
};
}
