#include <iostream>
#include <fstream>
// #include <boost/functional/hash.hpp>
#include <regex>
#include <unordered_set>
#include <set>
#include <list>
#include <unordered_map>
#include <algorithm>
#include <stdexcept>
#include <queue>
#include <time.h>

#define SYMBOLS 0
#define INITIAL 1
#define GOAL 2
#define ACTIONS 3
#define ACTION_DEFINITION 4
#define ACTION_PRECONDITION 5
#define ACTION_EFFECT 6
#define HEURISTIC 2

/*
HEURISTIC definition:
0 : no heuristic
1 : inadmissible heuristic
2 : admissible heuristic
*/

class GroundedCondition;
class Condition;
class GroundedAction;
class Action;
class Env;

using namespace std;

bool print_status = true;

class GroundedCondition
{
private:
    string predicate;
    list<string> arg_values;
    bool truth = true;

public:
    GroundedCondition(string predicate, list<string> arg_values, bool truth = true)
    {
        this->predicate = predicate;
        this->truth = truth;  // fixed
        for (string l : arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    GroundedCondition(const GroundedCondition& gc)
    {
        this->predicate = gc.predicate;
        this->truth = gc.truth;  // fixed
        for (string l : gc.arg_values)
        {
            this->arg_values.push_back(l);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }
    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const GroundedCondition& pred)
    {
        os << pred.toString() << " ";
        return os;
    }

    bool operator==(const GroundedCondition& rhs) const
    {
        if (this->predicate != rhs.predicate || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth()) // fixed
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        temp += this->predicate;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct GroundedConditionComparator
{
    bool operator()(const GroundedCondition& lhs, const GroundedCondition& rhs) const
    {
        return lhs == rhs;
    }
};

struct GroundedConditionHasher
{
    size_t operator()(const GroundedCondition& gcond) const
    {
        return hash<string>{}(gcond.toString());
    }
};

class Condition
{
private:
    string predicate;
    list<string> args;
    bool truth;

public:
    Condition(string pred, list<string> args, bool truth)
    {
        this->predicate = pred;
        this->truth = truth;
        for (string ar : args)
        {
            this->args.push_back(ar);
        }
    }

    string get_predicate() const
    {
        return this->predicate;
    }

    list<string> get_args() const
    {
        return this->args;
    }

    bool get_truth() const
    {
        return this->truth;
    }

    friend ostream& operator<<(ostream& os, const Condition& cond)
    {
        os << cond.toString() << " ";
        return os;
    }

    bool operator==(const Condition& rhs) const // fixed
    {

        if (this->predicate != rhs.predicate || this->args.size() != rhs.args.size())
            return false;

        auto lhs_it = this->args.begin();
        auto rhs_it = rhs.args.begin();

        while (lhs_it != this->args.end() && rhs_it != rhs.args.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }

        if (this->truth != rhs.get_truth())
            return false;

        return true;
    }

    string toString() const
    {
        string temp = "";
        if (!this->truth)
            temp += "!";
        temp += this->predicate;
        temp += "(";
        for (string l : this->args)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }

    bool change_truth()
    {
        this->truth = !(this->truth);
    }
};

struct ConditionComparator
{
    bool operator()(const Condition& lhs, const Condition& rhs) const
    {
        return lhs == rhs;
    }
};

struct ConditionHasher
{
    size_t operator()(const Condition& cond) const
    {
        return hash<string>{}(cond.toString());
    }
};

class Action
{
private:
    string name;
    list<string> args;
    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;

public:
    Action(string name, list<string> args,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& preconditions,
        unordered_set<Condition, ConditionHasher, ConditionComparator>& effects)
    {
        this->name = name;
        for (string l : args)
        {
            this->args.push_back(l);
        }
        for (Condition pc : preconditions)
        {
            this->preconditions.insert(pc);
        }
        for (Condition pc : effects)
        {
            this->effects.insert(pc);
        }
    }
    string get_name() const
    {
        return this->name;
    }
    list<string> get_args() const
    {
        return this->args;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_preconditions() const
    {
        return this->preconditions;
    }
    unordered_set<Condition, ConditionHasher, ConditionComparator> get_effects() const
    {
        return this->effects;
    }

    bool operator==(const Action& rhs) const
    {
        if (this->get_name() != rhs.get_name() || this->get_args().size() != rhs.get_args().size())
            return false;

        return true;
    }

    friend ostream& operator<<(ostream& os, const Action& ac)
    {
        os << ac.toString() << endl;
        os << "Precondition: ";
        for (Condition precond : ac.get_preconditions())
            os << precond;
        os << endl;
        os << "Effect: ";
        for (Condition effect : ac.get_effects())
            os << effect;
        os << endl;
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->get_name();
        temp += "(";
        for (string l : this->get_args())
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

struct ActionComparator
{
    bool operator()(const Action& lhs, const Action& rhs) const
    {
        return lhs == rhs;
    }
};

struct ActionHasher
{
    size_t operator()(const Action& ac) const
    {
        return hash<string>{}(ac.get_name());
    }
};

class Env
{
private:
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> initial_conditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> goal_conditions;
    unordered_set<Action, ActionHasher, ActionComparator> actions;
    unordered_set<string> symbols;

public:
    void remove_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.erase(gc);
    }
    void add_initial_condition(GroundedCondition gc)
    {
        this->initial_conditions.insert(gc);
    }
    void add_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.insert(gc);
    }
    void remove_goal_condition(GroundedCondition gc)
    {
        this->goal_conditions.erase(gc);
    }
    void add_symbol(string symbol)
    {
        symbols.insert(symbol);
    }
    void add_symbols(list<string> symbols)
    {
        for (string l : symbols)
            this->symbols.insert(l);
    }
    void add_action(Action action)
    {
        this->actions.insert(action);
    }

    Action get_action(string name)
    {
        for (Action a : this->actions)
        {
            if (a.get_name() == name)
                return a;
        }
        throw runtime_error("Action " + name + " not found!");
    }

    unordered_set<string> get_symbols() const
    {
        return this->symbols;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_initial_conditions() const
    {
        return this->initial_conditions;
    }

    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> get_goal_conditions() const
    {
        return this->goal_conditions;
    }

    unordered_set<Action, ActionHasher, ActionComparator> get_actions() const
    {
        return this->actions;
    }

    friend ostream& operator<<(ostream& os, const Env& w)
    {
        os << "***** Environment *****" << endl << endl;
        os << "Symbols: ";
        for (string s : w.get_symbols())
            os << s + ",";
        os << endl;
        os << "Initial conditions: ";
        for (GroundedCondition s : w.initial_conditions)
            os << s;
        os << endl;
        os << "Goal conditions: ";
        for (GroundedCondition g : w.goal_conditions)
            os << g;
        os << endl;
        os << "Actions:" << endl;
        for (Action g : w.actions)
            os << g << endl;
        cout << "***** Environment Created! *****" << endl;
        return os;
    }
};

class GroundedAction
{
private:
    string name;
    list<string> arg_values;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gPreconditions;
    unordered_set<GroundedCondition, GroundedConditionHasher, GroundedConditionComparator> gEffects;

public:
    GroundedAction(string name, list<string> arg_values)
    {
        this->name = name;
        for (string ar : arg_values)
        {
            this->arg_values.push_back(ar);
        }
    }

    string get_name() const
    {
        return this->name;
    }

    list<string> get_arg_values() const
    {
        return this->arg_values;
    }

    bool operator==(const GroundedAction& rhs) const
    {
        if (this->name != rhs.name || this->arg_values.size() != rhs.arg_values.size())
            return false;

        auto lhs_it = this->arg_values.begin();
        auto rhs_it = rhs.arg_values.begin();

        while (lhs_it != this->arg_values.end() && rhs_it != rhs.arg_values.end())
        {
            if (*lhs_it != *rhs_it)
                return false;
            ++lhs_it;
            ++rhs_it;
        }
        return true;
    }

    friend ostream& operator<<(ostream& os, const GroundedAction& gac)
    {
        os << gac.toString() << " ";
        return os;
    }

    string toString() const
    {
        string temp = "";
        temp += this->name;
        temp += "(";
        for (string l : this->arg_values)
        {
            temp += l + ",";
        }
        temp = temp.substr(0, temp.length() - 1);
        temp += ")";
        return temp;
    }
};

list<string> parse_symbols(string symbols_str)
{
    list<string> symbols;
    size_t pos = 0;
    string delimiter = ",";
    while ((pos = symbols_str.find(delimiter)) != string::npos)
    {
        string symbol = symbols_str.substr(0, pos);
        symbols_str.erase(0, pos + delimiter.length());
        symbols.push_back(symbol);
    }
    symbols.push_back(symbols_str);
    return symbols;
}

Env* create_env(char* filename)
{
    ifstream input_file(filename);
    Env* env = new Env();
    regex symbolStateRegex("symbols:", regex::icase);
    regex symbolRegex("([a-zA-Z0-9_, ]+) *");
    regex initialConditionRegex("initialconditions:(.*)", regex::icase);
    regex conditionRegex("(!?[A-Z][a-zA-Z_]*) *\\( *([a-zA-Z0-9_, ]+) *\\)");
    regex goalConditionRegex("goalconditions:(.*)", regex::icase);
    regex actionRegex("actions:", regex::icase);
    regex precondRegex("preconditions:(.*)", regex::icase);
    regex effectRegex("effects:(.*)", regex::icase);
    int parser = SYMBOLS;

    unordered_set<Condition, ConditionHasher, ConditionComparator> preconditions;
    unordered_set<Condition, ConditionHasher, ConditionComparator> effects;
    string action_name;
    string action_args;

    string line;
    if (input_file.is_open())
    {
        while (getline(input_file, line))
        {
            string::iterator end_pos = remove(line.begin(), line.end(), ' ');
            line.erase(end_pos, line.end());

            if (line == "")
                continue;

            if (parser == SYMBOLS)
            {
                smatch results;
                if (regex_search(line, results, symbolStateRegex))
                {
                    line = line.substr(8);
                    sregex_token_iterator iter(line.begin(), line.end(), symbolRegex, 0);
                    sregex_token_iterator end;

                    env->add_symbols(parse_symbols(iter->str()));  // fixed

                    parser = INITIAL;
                }
                else
                {
                    cout << "Symbols are not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == INITIAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, initialConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_initial_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_initial_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = GOAL;
                }
                else
                {
                    cout << "Initial conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == GOAL)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, goalConditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        if (predicate[0] == '!')
                        {
                            env->remove_goal_condition(
                                GroundedCondition(predicate.substr(1), parse_symbols(args)));
                        }
                        else
                        {
                            env->add_goal_condition(
                                GroundedCondition(predicate, parse_symbols(args)));
                        }
                    }

                    parser = ACTIONS;
                }
                else
                {
                    cout << "Goal conditions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTIONS)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, actionRegex))
                {
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Actions not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_DEFINITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, conditionRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;
                    // name
                    action_name = iter->str();
                    iter++;
                    // args
                    action_args = iter->str();
                    iter++;

                    parser = ACTION_PRECONDITION;
                }
                else
                {   
                    cout << "Action not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_PRECONDITION)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, precondRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition precond(predicate, parse_symbols(args), truth);
                        preconditions.insert(precond);
                    }

                    parser = ACTION_EFFECT;
                }
                else
                {
                    cout << "Precondition not specified correctly." << endl;
                    throw;
                }
            }
            else if (parser == ACTION_EFFECT)
            {
                const char* line_c = line.c_str();
                if (regex_match(line_c, effectRegex))
                {
                    const std::vector<int> submatches = { 1, 2 };
                    sregex_token_iterator iter(
                        line.begin(), line.end(), conditionRegex, submatches);
                    sregex_token_iterator end;

                    while (iter != end)
                    {
                        // name
                        string predicate = iter->str();
                        iter++;
                        // args
                        string args = iter->str();
                        iter++;

                        bool truth;

                        if (predicate[0] == '!')
                        {
                            predicate = predicate.substr(1);
                            truth = false;
                        }
                        else
                        {
                            truth = true;
                        }

                        Condition effect(predicate, parse_symbols(args), truth);
                        effects.insert(effect);
                    }

                    env->add_action(
                        Action(action_name, parse_symbols(action_args), preconditions, effects));

                    preconditions.clear();
                    effects.clear();
                    parser = ACTION_DEFINITION;
                }
                else
                {
                    cout << "Effects not specified correctly." << endl;
                    throw;
                }
            }
        }
        input_file.close();
    }

    else
        cout << "Unable to open file";

    return env;
}

class State{
public:
    unordered_set<Condition, ConditionHasher, ConditionComparator> conditions;
    GroundedAction* gAction = NULL;
    State* parent = NULL;
    int cost = 0;
    int h = 0;
    int closed = false;

    string get_name(){
       string name;
        for (auto i : conditions){
            name += i.toString();
        } 
        return name;
    }

    void set_action(const Action& action){
        delete gAction;
        gAction = new GroundedAction(action.get_name(), action.get_args());
    }
};

size_t state_key(unordered_set<Condition, ConditionHasher, ConditionComparator>& conditions){
    ConditionHasher cHasher;
    size_t key;
    for (auto i : conditions){
        key += cHasher(i);
    }
    return key;
}

bool reached_goal(State* currentStatePtr, State* goalStatePtr){
    int satisfiedConditions = 0;
    for (auto i : goalStatePtr->conditions){
        if((currentStatePtr->conditions).find(i) != (currentStatePtr->conditions).end()){
            satisfiedConditions++;
        }
    }
    if (satisfiedConditions == (goalStatePtr->conditions).size()){
        return true;
    }
    return false;
}

void loopFunction(unordered_map<string, string>& substituteSymbols,const Action& action, vector<Action>& possibleActions, Env* env, list<string>::iterator it, int level, vector<string>& chosenSymbols){
    if (level == (action.get_args()).size()){
        //we have all the substitute symbols now.
        list<string> possibleArgs;
        unordered_set<Condition, ConditionHasher, ConditionComparator> possiblePreconditions;
        unordered_set<Condition, ConditionHasher, ConditionComparator> possibleEffects;

        //form all the new arguments.
        for (auto arg : action.get_args()){
            possibleArgs.push_back(substituteSymbols[arg]);
        }

        //form all the new preconditions
        for (auto precondition : action.get_preconditions()){
            list<string> possiblePreconditionArgs;
            for (auto arg : precondition.get_args()){
                if (substituteSymbols.find(arg) != substituteSymbols.end()){
                    possiblePreconditionArgs.push_back(substituteSymbols[arg]);
                }
                else{
                    possiblePreconditionArgs.push_back(arg);
                }
            }
            possiblePreconditions.insert(Condition(precondition.get_predicate(), possiblePreconditionArgs, precondition.get_truth()));
        }

        //form all the effects.
        for (auto effect : action.get_effects()){
            list<string> possibleEffectArgs;
            for(auto arg : effect.get_args()){
                if (substituteSymbols.find(arg) != substituteSymbols.end()){
                    possibleEffectArgs.push_back(substituteSymbols[arg]);
                }
                else{
                    possibleEffectArgs.push_back(arg);
                }
            }
            possibleEffects.insert(Condition(effect.get_predicate(), possibleEffectArgs, effect.get_truth()));
        }

        //form a new action and push it into possible actions.
        possibleActions.push_back(Action(action.get_name(), possibleArgs, possiblePreconditions, possibleEffects));

        return;
    }

    else{
        for (auto symbol : env->get_symbols()){
            if (find(chosenSymbols.begin(), chosenSymbols.end(), symbol) == chosenSymbols.end()){
                substituteSymbols[*it] = symbol;
                chosenSymbols.push_back(symbol);
                loopFunction(substituteSymbols, action, possibleActions, env, next(it,1), level+1, chosenSymbols);
                substituteSymbols.erase(*it);
                chosenSymbols.pop_back();
            }
        }
    }
}

Condition convertToConditions(GroundedCondition& gCondition){
    return Condition(gCondition.get_predicate(), gCondition.get_arg_values(), gCondition.get_truth());
}

//given a state pointer, check if the aciton can be applied or not. If it can be applied, modify the statePtr.
bool apply_action(const Action& action, unordered_set<Condition, ConditionHasher, ConditionComparator>& conditions, bool calculatingHeuristic = false){
    int satisfiedConditions = 0;
    //check if the preconditions are satisfied.
    for (auto i : action.get_preconditions()){
        if (i.get_truth()){
            if(conditions.find(i) != conditions.end()){
                satisfiedConditions++;
            }
        }
        else{
            Condition dummyCondition(i.get_predicate(), i.get_args(), i.get_truth());
            dummyCondition.change_truth();
            if(conditions.find(dummyCondition) == conditions.end()){
                satisfiedConditions++;
            }
        }
    }

    if (satisfiedConditions != (action.get_preconditions()).size()){
        return false;
    }

    //if the preconditions are satisfied, we need to apply the effects of the action.
    for (auto i : action.get_effects()){
        //if the effect is negating a condition
        if (!i.get_truth()){
            //erase negative conditions when not calculating heuristic.
            if (!calculatingHeuristic){
                Condition dummyCondition(i.get_predicate(), i.get_args(), i.get_truth());
                dummyCondition.change_truth();
                //this dummyCondition needs to be removed from the state conditions.
                conditions.erase(dummyCondition);
            }
        }
        else{
            conditions.insert(i);
        }
    }
    //return true if reached here.
    return true;
}

struct CompareF{
    bool operator()(const State* lhs, const State* rhs) const
    {
        return (lhs->cost + lhs->h) > (rhs->cost + rhs->h);
    }
};

int calculate_heuristic(int heuristic_id, State* initialStatePtr, State* goalStatePtr, const vector<Action>& possibleActions){
    //no heuristic
    if (heuristic_id == 0){
        return 0;
    }

    //inadmissible heuristic
    if (heuristic_id == 1){
        return max((int)(initialStatePtr->conditions).size() - (int)(goalStatePtr->conditions).size(),0);
    }

    //admissible heuristic
    if (heuristic_id == 2){
        State* currentStatePtr;
        State* neighborStatePtr;
        State* initialStatePtr_2 = new State();
        priority_queue<State*, vector<State*>, CompareF> openList;
        unordered_map<size_t, State*> openPtrList;
        unordered_set<Condition, ConditionHasher, ConditionComparator> conditions;
        bool actionPossible = false;
        size_t neighborKey;
        size_t currentKey;
        int heuristic;

        initialStatePtr_2->parent = NULL;
        initialStatePtr_2->cost = 0;
        initialStatePtr_2->conditions = initialStatePtr->conditions;
        initialStatePtr_2->closed = false;

        openList.push(initialStatePtr_2);
        //now we can begin planning.
        while(!openList.empty()){
            currentStatePtr = openList.top();
            openList.pop();

            //if the state is closed, continue.
            if (currentStatePtr->closed){
                continue;
            }

            //close the popped pointer.
            currentStatePtr->closed = true;
            
            //check if we have reached the goal state        
            if (reached_goal(currentStatePtr, goalStatePtr)){
                break;
            }

            //loop through the possible actions
            for (auto action : possibleActions){
                conditions = currentStatePtr->conditions;
                //is the action possible?
                actionPossible = apply_action(action, conditions, true);

                //if the actions is possible
                if (actionPossible){
                    //conditions would be updated to new conditions.
                    neighborKey = state_key(conditions);
                    //check if the neighbor has been visited before
                    if (openPtrList.find(neighborKey) != openPtrList.end()){
                        //if the current key has been closed, continue
                        if ((openPtrList[neighborKey])->closed){
                            continue;
                        }
                        neighborStatePtr = openPtrList[neighborKey];
                        if(neighborStatePtr->cost > currentStatePtr->cost + 1){
                            neighborStatePtr->cost = currentStatePtr->cost + 1;
                            neighborStatePtr->parent = currentStatePtr;
                            openList.push(neighborStatePtr);
                        }
                    }
                    else{
                        neighborStatePtr = new State();
                        neighborStatePtr->conditions = conditions;
                        neighborStatePtr->cost = currentStatePtr->cost + 1;
                        neighborStatePtr->parent = currentStatePtr;
                        openList.push(neighborStatePtr);
                        openPtrList[neighborKey] = neighborStatePtr;
                    }
                }
            }
        }

        heuristic = currentStatePtr->cost;
        // delete all elements in openPtrList.
        auto it = openPtrList.begin();
        while(it != openPtrList.end()){
            if (it->second == currentStatePtr){
            }
            delete (it->second);
            it++;
        }
        //delete initialStatePtr_2.
        delete initialStatePtr_2;

        return heuristic;
    }
}

list<GroundedAction> planner(Env* env)
{
    State* initialStatePtr = new State();
    State* goalStatePtr = new State();
    State* currentStatePtr;
    State* neighborStatePtr;
    vector<Action> possibleActions;
    vector<string> chosenSymbols;
    list<GroundedAction> plannedActions;
    priority_queue<State*, vector<State*>, CompareF> openList;
    unordered_map<string, string> substituteSymbols;
    unordered_map<size_t, State*> openPtrList;
    unordered_set<Condition, ConditionHasher, ConditionComparator> conditions;
    bool actionPossible = false;
    int steps = 0;
    size_t neighborKey;

    //convert all grounded conditions to condition and store it in the corresponding pointers.
    for (auto i : env->get_initial_conditions()){
        initialStatePtr->conditions.insert(convertToConditions(i));
    }

    for (auto j : env->get_goal_conditions()){
        goalStatePtr->conditions.insert(convertToConditions(j));
    }

    //iterate through all the actions in the environment and generate all possible actions.
    for (auto action : env->get_actions()){
       //create all possible combinations of symbols for this action. New actions should be in possibleActions.
        loopFunction(substituteSymbols, action, possibleActions, env, (action.get_args()).begin(), 0, chosenSymbols);
        chosenSymbols.clear();
        substituteSymbols.clear();
    }

    openList.push(initialStatePtr);
    //now we can begin planning.
    while(!openList.empty()){
        steps++;
        currentStatePtr = openList.top();
        openList.pop();

        //if the state is closed, continue.
        if (currentStatePtr->closed){
            continue;
        }

        //close the popped pointer.
        currentStatePtr->closed = true;
        
        //check if we have reached the goal state        
        if (reached_goal(currentStatePtr, goalStatePtr)){
            break;
        }

        //loop through the possible actions
        for (auto action : possibleActions){
            conditions = currentStatePtr->conditions;
            //is the action possible?
            actionPossible = apply_action(action, conditions, false);

            //if the actions is possible
            if (actionPossible){
                //conditions would be updated to new conditions.
                neighborKey = state_key(conditions);
                //check if the neighbor has been visited before
                if (openPtrList.find(neighborKey) != openPtrList.end()){
                    //if the current key has been closed, continue
                    if ((openPtrList[neighborKey])->closed){
                        continue;
                    }
                    neighborStatePtr = openPtrList[neighborKey];
                    if(neighborStatePtr->cost > currentStatePtr->cost + 1){
                        neighborStatePtr->cost = currentStatePtr->cost + 1;
                        neighborStatePtr->set_action(action);
                        neighborStatePtr->parent = currentStatePtr;
                        openList.push(neighborStatePtr);
                    }
                }
                else{
                    neighborStatePtr = new State();
                    neighborStatePtr->conditions = conditions;
                    neighborStatePtr->cost = currentStatePtr->cost + 1;
                    neighborStatePtr->set_action(action);
                    neighborStatePtr->parent = currentStatePtr;
                    neighborStatePtr->h = calculate_heuristic(HEURISTIC, neighborStatePtr, goalStatePtr, possibleActions);
                    openList.push(neighborStatePtr);
                    openPtrList[neighborKey] = neighborStatePtr;
                }
            }
        }
    }
    
    //currentStatePtr must be the goal pointer
    while(currentStatePtr->parent){
        plannedActions.push_back(*(currentStatePtr->gAction));
        currentStatePtr = currentStatePtr->parent;
    }
    plannedActions.reverse();

    cout << "number of expansions "<< steps << endl;

    return plannedActions;
}

int main(int argc, char* argv[])
{
    // DO NOT CHANGE THIS FUNCTION
    char* filename = (char*)("example.txt");
    if (argc > 1)
        filename = argv[1];

    cout << "Environment: " << filename << endl << endl;
    Env* env = create_env(filename);
    if (print_status)
    {
        cout << *env;
    }

    time_t start = time(NULL);
    list<GroundedAction> actions = planner(env);
    time_t end = time(NULL);

    cout << "time taken is " << end-start << endl;

    cout << "\nPlan: " << endl;
    for (GroundedAction gac : actions)
    {
        cout << gac << endl;
    }

    return 0;
}