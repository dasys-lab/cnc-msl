#include "fann.h"

int main()
{

    unsigned int layers[4] = {2, 15, 1};

    const float desired_error = (const float) 0.002;
    const unsigned int max_epochs = 200;
    const unsigned int epochs_between_reports = 10;

    //struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);
    struct fann *ann = fann_create_standard_array(3, layers);
    fann_set_activation_function_hidden(ann, FANN_GAUSSIAN);
    fann_set_activation_function_output(ann, FANN_LINEAR);
    //fann_set_activation_function_layer(ann, FANN_SIGMOID_SYMMETRIC, 0);
    //fann_set_activation_function_layer(ann, FANN_SIGMOID_SYMMETRIC, 1);
    //fann_set_activation_function_layer(ann, FANN_SIGMOID_SYMMETRIC, 2);
    //fann_set_activation_function_layer(ann, FANN_SIGMOID_SYMMETRIC, 3);

    fann_set_activation_steepness_hidden(ann, 0.5);
    fann_set_training_algorithm(ann, FANN_TRAIN_RPROP);

    fann_train_on_file(ann, "ball.data", max_epochs, epochs_between_reports, desired_error);

    fann_save(ann, "COI.net");

    fann_destroy(ann);

    return 0;
}
