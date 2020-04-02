#include <rosrect-listener-agent/backend_api.h>

int main(int argc, char* argv[]) {
    
    std::cout << "Testing API Login..." << std::endl;

    // Instantiate BackendApi class object. This will automatically login to our API and 
    // store access tokens for creating tickets.
    BackendApi api_instance = BackendApi();
    
    std::cout << "Finished testing API Login..." << std::endl;

    std::cout << "Testing ticket creation..." << std::endl;

    // Create a sample ticket using the api object above. 
    // Sample ticket details
    ticketDetails test_ticket;

    test_ticket.module_name = "Tester";
    test_ticket.error_text = "Aborted. Cannot recover.";
    test_ticket.agent_id = std::getenv("AGENT_CODE");
    test_ticket.robot_id = std::getenv("ROBOT_CODE");
    test_ticket.prop_id = std::getenv("SITE_CODE");

    // In this case, we call wait. 
    // However, you won’t usually want to wait for the asynchronous operations (this errors if wait is not used)
    api_instance.create_ticket(test_ticket).wait();
    
    std::cout << "Finished testing ticket creation..." << std::endl;
        
    return 0;
}