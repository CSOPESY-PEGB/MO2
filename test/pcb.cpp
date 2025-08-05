#include "pcb.hpp"

const std::string pcb::storage_file = "storage.txt";

// Atom Serialization
void Atom::write(std::ofstream& out) const {
    out.write(reinterpret_cast<const char*>(&type), sizeof(type));
    size_t len = string_value.length();
    out.write(reinterpret_cast<const char*>(&len), sizeof(len));
    out.write(string_value.c_str(), len);
    out.write(reinterpret_cast<const char*>(&number_value), sizeof(number_value));
}

// Atom Deserialization
void Atom::read(std::ifstream& in) {
    in.read(reinterpret_cast<char*>(&type), sizeof(type));
    size_t len;
    in.read(reinterpret_cast<char*>(&len), sizeof(len));
    string_value.resize(len);
    in.read(&string_value[0], len);
    in.read(reinterpret_cast<char*>(&number_value), sizeof(number_value));
}

// Expr copy constructor
Expr::Expr(const Expr& other)
    : type(other.type), var_name(other.var_name), body(other.body) {
    if (other.atom_value) atom_value = std::make_unique<Atom>(*other.atom_value);
    if (other.lhs) lhs = std::make_unique<Atom>(*other.lhs);
    if (other.rhs) rhs = std::make_unique<Atom>(*other.rhs);
    if (other.n) n = std::make_unique<Atom>(*other.n);
}

// Expr copy assignment
Expr& Expr::operator=(const Expr& other) {
    if (this != &other) {
        type = other.type;
        var_name = other.var_name;
        body = other.body;
        atom_value.reset();
        if (other.atom_value) atom_value = std::make_unique<Atom>(*other.atom_value);
        lhs.reset();
        if (other.lhs) lhs = std::make_unique<Atom>(*other.lhs);
        rhs.reset();
        if (other.rhs) rhs = std::make_unique<Atom>(*other.rhs);
        n.reset();
        if (other.n) n = std::make_unique<Atom>(*other.n);
    }
    return *this;
}

// Expr move constructor
Expr::Expr(Expr&& other) noexcept
    : type(other.type), var_name(std::move(other.var_name)),
      atom_value(std::move(other.atom_value)),
      lhs(std::move(other.lhs)), rhs(std::move(other.rhs)),
      n(std::move(other.n)), body(std::move(other.body)) {}

// Expr move assignment
Expr& Expr::operator=(Expr&& other) noexcept {
    if (this != &other) {
        type = other.type;
        var_name = std::move(other.var_name);
        atom_value = std::move(other.atom_value);
        lhs = std::move(other.lhs);
        rhs = std::move(other.rhs);
        n = std::move(other.n);
        body = std::move(other.body);
    }
    return *this;
}

// Expr factory methods
Expr Expr::make_declare(std::string name, std::unique_ptr<Atom> value) {
    Expr e(DECLARE);
    e.var_name = std::move(name);
    e.atom_value = std::move(value);
    return e;
}

Expr Expr::make_call(std::string name, std::unique_ptr<Atom> arg) {
    Expr e(CALL);
    e.var_name = std::move(name);
    e.atom_value = std::move(arg);
    return e;
}

Expr Expr::make_call_concat(std::string name, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs) {
    Expr e(CALL);
    e.var_name = std::move(name);
    e.lhs = std::move(lhs);
    e.rhs = std::move(rhs);
    return e;
}

Expr Expr::make_add(std::string var, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs) {
    Expr e(ADD);
    e.var_name = std::move(var);
    e.lhs = std::move(lhs);
    e.rhs = std::move(rhs);
    return e;
}

Expr Expr::make_sub(std::string var, std::unique_ptr<Atom> lhs, std::unique_ptr<Atom> rhs) {
    Expr e(SUB);
    e.var_name = std::move(var);
    e.lhs = std::move(lhs);
    e.rhs = std::move(rhs);
    return e;
}

Expr Expr::make_for(std::vector<Expr> body, std::unique_ptr<Atom> n) {
    Expr e(FOR);
    e.body = std::move(body);
    e.n = std::move(n);
    return e;
}

Expr Expr::make_constant(std::unique_ptr<Atom> value) {
    Expr e(CONSTANT);
    e.atom_value = std::move(value);
    return e;
}

Expr Expr::make_read(std::string var_name, std::unique_ptr<Atom> address) {
    Expr e(READ);
    e.var_name = std::move(var_name);
    e.atom_value = std::move(address);
    return e;
}

Expr Expr::make_write(std::unique_ptr<Atom> address, std::unique_ptr<Atom> value) {
    Expr e(WRITE);
    e.lhs = std::move(address);
    e.rhs = std::move(value);
    return e;
}

// Expr Serialization
void Expr::write(std::ofstream& out) const {
    out.write(reinterpret_cast<const char*>(&type), sizeof(type));
    size_t var_name_len = var_name.length();
    out.write(reinterpret_cast<const char*>(&var_name_len), sizeof(var_name_len));
    out.write(var_name.c_str(), var_name_len);

    bool has_atom_value = atom_value != nullptr;
    out.write(reinterpret_cast<const char*>(&has_atom_value), sizeof(has_atom_value));
    if (atom_value) atom_value->write(out);

    bool has_lhs = lhs != nullptr;
    out.write(reinterpret_cast<const char*>(&has_lhs), sizeof(has_lhs));
    if (lhs) lhs->write(out);

    bool has_rhs = rhs != nullptr;
    out.write(reinterpret_cast<const char*>(&has_rhs), sizeof(has_rhs));
    if (rhs) rhs->write(out);

    bool has_n = n != nullptr;
    out.write(reinterpret_cast<const char*>(&has_n), sizeof(has_n));
    if (n) n->write(out);

    size_t body_size = body.size();
    out.write(reinterpret_cast<const char*>(&body_size), sizeof(body_size));
    for (const auto& expr : body) {
        expr.write(out);
    }
}

// Expr Deserialization
void Expr::read(std::ifstream& in) {
    in.read(reinterpret_cast<char*>(&type), sizeof(type));
    size_t var_name_len;
    in.read(reinterpret_cast<char*>(&var_name_len), sizeof(var_name_len));
    var_name.resize(var_name_len);
    in.read(&var_name[0], var_name_len);

    bool has_atom_value;
    in.read(reinterpret_cast<char*>(&has_atom_value), sizeof(has_atom_value));
    if (has_atom_value) {
        atom_value = std::make_unique<Atom>();
        atom_value->read(in);
    }

    bool has_lhs;
    in.read(reinterpret_cast<char*>(&has_lhs), sizeof(has_lhs));
    if (has_lhs) {
        lhs = std::make_unique<Atom>();
        lhs->read(in);
    }

    bool has_rhs;
    in.read(reinterpret_cast<char*>(&has_rhs), sizeof(has_rhs));
    if (has_rhs) {
        rhs = std::make_unique<Atom>();
        rhs->read(in);
    }

    bool has_n;
    in.read(reinterpret_cast<char*>(&has_n), sizeof(has_n));
    if (has_n) {
        n = std::make_unique<Atom>();
        n->read(in);
    }

    size_t body_size;
    in.read(reinterpret_cast<char*>(&body_size), sizeof(body_size));
    body.resize(body_size);
    for (size_t i = 0; i < body_size; ++i) {
        body[i].read(in);
    }
}


// pcb store method
void pcb::store() {
    std::ofstream out(storage_file, std::ios::binary | std::ios::app);
    if (!out) {
        throw std::runtime_error("Cannot open storage file for writing.");
    }

    // Write PCB ID
    out.write(reinterpret_cast<const char*>(&pcb_id), sizeof(pcb_id));

    // Write program size
    size_t program_size = program.size();
    out.write(reinterpret_cast<const char*>(&program_size), sizeof(program_size));

    // Write program instructions
    for (const auto& instruction : program) {
        uint8_t type = static_cast<uint8_t>(instruction.index());
        out.write(reinterpret_cast<const char*>(&type), sizeof(type));

        if (std::holds_alternative<uint16_t>(instruction)) {
            uint16_t val = std::get<uint16_t>(instruction);
            out.write(reinterpret_cast<const char*>(&val), sizeof(val));
        } else if (std::holds_alternative<Expr>(instruction)) {
            const Expr& expr = std::get<Expr>(instruction);
            expr.write(out);
        }
    }
}

// pcb load method
bool pcb::load() {
    std::ifstream in(storage_file, std::ios::binary);
    if (!in) {
        return false; // File doesn't exist or cannot be opened.
    }

    while (in.peek() != EOF) {
        uint32_t current_id;
        in.read(reinterpret_cast<char*>(&current_id), sizeof(current_id));

        if (in.gcount() == 0) break; // End of file

        size_t program_size;
        in.read(reinterpret_cast<char*>(&program_size), sizeof(program_size));

        if (current_id == this->pcb_id) {
            program.clear();
            for (size_t i = 0; i < program_size; ++i) {
                uint8_t type;
                in.read(reinterpret_cast<char*>(&type), sizeof(type));

                if (type == 0) { // uint16_t
                    uint16_t val;
                    in.read(reinterpret_cast<char*>(&val), sizeof(val));
                    program.emplace_back(val);
                } else if (type == 1) { // Expr
                    Expr expr;
                    expr.read(in);
                    program.emplace_back(std::move(expr));
                }
            }
            return true; // Found and loaded
        } else {
            // Skip this PCB's data
            for (size_t i = 0; i < program_size; ++i) {
                uint8_t type;
                in.read(reinterpret_cast<char*>(&type), sizeof(type));
                if (type == 0) {
                    in.seekg(sizeof(uint16_t), std::ios::cur);
                } else if (type == 1) {
                    // This is tricky, we need to deserialize to skip correctly.
                    // A better format would have stored the size of the block.
                    // For now, we deserialize to a dummy object.
                    Expr dummy;
                    dummy.read(in);
                }
            }
        }
    }
    return false; // Not found
}
