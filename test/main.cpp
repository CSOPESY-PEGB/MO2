#include "pcb.hpp"
#include <cassert>
#include <iostream>
#include <cstdio> // For remove()

// Helper to compare two Atoms
bool compare_atoms(const Atom* a, const Atom* b) {
    if (!a && !b) return true;
    if (!a || !b) return false;
    if (a->type != b->type) return false;
    if (a->type == Atom::NUMBER) {
        return a->number_value == b->number_value;
    } else {
        return a->string_value == b->string_value;
    }
}

// Helper to compare two Exprs
bool compare_exprs(const Expr& a, const Expr& b) {
    if (a.type != b.type) return false;
    if (a.var_name != b.var_name) return false;
    if (!compare_atoms(a.atom_value.get(), b.atom_value.get())) return false;
    if (!compare_atoms(a.lhs.get(), b.lhs.get())) return false;
    if (!compare_atoms(a.rhs.get(), b.rhs.get())) return false;
    if (!compare_atoms(a.n.get(), b.n.get())) return false;
    if (a.body.size() != b.body.size()) return false;
    for (size_t i = 0; i < a.body.size(); ++i) {
        if (!compare_exprs(a.body[i], b.body[i])) return false;
    }
    return true;
}


void run_tests() {
    std::cout << "Starting tests..." << std::endl;

    // Clean up previous test file
    remove("storage.txt");

    // Test 1: Create and store PCB 1
    pcb pcb1(101);
    pcb1.add_instruction(static_cast<uint16_t>(12345));
    pcb1.add_instruction(Expr::make_declare("my_var", std::make_unique<Atom>(55)));
    
    std::vector<Expr> for_body;
    for_body.push_back(Expr::make_write(std::make_unique<Atom>(100), std::make_unique<Atom>("hello", Atom::STRING)));
    pcb1.add_instruction(Expr::make_for(std::move(for_body), std::make_unique<Atom>(3)));

    std::cout << "Storing PCB 1..." << std::endl;
    pcb1.store();

    // Test 2: Create and store PCB 2
    pcb pcb2(202);
    pcb2.add_instruction(Expr::make_add("result", std::make_unique<Atom>(10), std::make_unique<Atom>(20)));
    pcb2.add_instruction(static_cast<uint16_t>(54321));

    std::cout << "Storing PCB 2..." << std::endl;
    pcb2.store();

    // Test 3: Load and verify PCB 1
    pcb pcb1_loaded(101);
    bool loaded1 = pcb1_loaded.load();
    assert(loaded1);
    std::cout << "PCB 1 loaded successfully." << std::endl;

    const auto& prog1_orig = pcb1.get_program();
    const auto& prog1_loaded = pcb1_loaded.get_program();
    assert(prog1_orig.size() == prog1_loaded.size());

    // Check instruction 0
    assert(std::get<uint16_t>(prog1_orig[0]) == std::get<uint16_t>(prog1_loaded[0]));
    // Check instruction 1
    assert(compare_exprs(std::get<Expr>(prog1_orig[1]), std::get<Expr>(prog1_loaded[1])));
    // Check instruction 2
    assert(compare_exprs(std::get<Expr>(prog1_orig[2]), std::get<Expr>(prog1_loaded[2])));
    std::cout << "PCB 1 data verified." << std::endl;


    // Test 4: Load and verify PCB 2
    pcb pcb2_loaded(202);
    bool loaded2 = pcb2_loaded.load();
    assert(loaded2);
    std::cout << "PCB 2 loaded successfully." << std::endl;

    const auto& prog2_orig = pcb2.get_program();
    const auto& prog2_loaded = pcb2_loaded.get_program();
    assert(prog2_orig.size() == prog2_loaded.size());

    // Check instruction 0
    assert(compare_exprs(std::get<Expr>(prog2_orig[0]), std::get<Expr>(prog2_loaded[0])));
    // Check instruction 1
    assert(std::get<uint16_t>(prog2_orig[1]) == std::get<uint16_t>(prog2_loaded[1]));
    std::cout << "PCB 2 data verified." << std::endl;


    // Test 5: Attempt to load a non-existent PCB
    pcb pcb3_nonexistent(999);
    bool loaded3 = pcb3_nonexistent.load();
    assert(!loaded3);
    std::cout << "Test for non-existent PCB passed." << std::endl;

    std::cout << "All tests passed!" << std::endl;

    // Clean up
    remove("storage.txt");
}

int main() {
    try {
        run_tests();
    } catch (const std::exception& e) {
        std::cerr << "An exception occurred: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
