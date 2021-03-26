#include <fdeep/fdeep.hpp>

int main()
{
    const auto model = fdeep::load_model("tf_model_driving_frugal.json");
    const auto result = model.predict(
        {fdeep::tensor(fdeep::tensor_shape(static_cast<std::size_t>(4)),
                       std::vector<float>{1, 2, 3, 4})});
    std::cout << fdeep::show_tensors(result) << std::endl;
    return 0;
}