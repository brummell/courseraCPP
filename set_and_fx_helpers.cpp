//
// Created by Brummell, Doug on 8/27/16.
//

#include <algorithm>

#from http://blog.madhukaraphatak.com/functional-programming-in-c++/
# TODO: implement/ensure implementation for Sets
template <typename Collection,typename unop>
void for_each(Collection col, unop op){
    std::for_each(col.begin(),col.end(),op);
}

template <typename Collection,typename unop>
Collection map(Collection col,unop op) {
    std::transform(col.begin(),col.end(),col.begin(),op);
    return col;
}

template <typename Collection,typename T>
bool exists(Collection col, M member) {
    std::any_of(col.begin(),col.end(),col.begin(),op);
    return col;
}

// In same way you can implement zip,exists combinators.
// All above combinators including zip and exists, do not change the size of the collection. But if you want to implement filter, you have to change the size.

No method in algorithm allows you to change the size of the collection. So we need to implement filter in following two steps

Determine the indexes inside the collection which do not satisfy predicates.
Remove those indexes from the vector.
Before implementing filter, we will implement filterNot. filterNot combinator removes all the elements which satisfies the predicate.

template <typename Collection,typename Predicate>
Collection filterNot(Collection col,Predicate predicate ) {
    auto returnIterator = std::remove_if(col.begin(),col.end(),predicate);
    col.erase(returnIterator,std::end(col));
    return col;
}
We have used erase function on vector to remove the elements that determined by remove_if function.
Now we can implement filter in terms of filterNot.
template <typename Collection,typename Predicate>
Collection filter(Collection col,Predicate predicate) {
    //capture the predicate in order to be used inside function
    auto fnCol = filterNot(col,[predicate](typename Collection::value_type i) { return !predicate(i);});
    return fnCol;
}
If you observe code carefully, you can see that we have captured that predicate in our lambda so that we can use it inside. If you don’t capture it, compiler will give an error.

Also as we don’t know the type expected by predicate, we can use Collection::value_type to say whatever the type of the collection elements it will be taken by the predicate. This makes our code highly generic.

Finally we can filter as follows

auto filteredCol = filter(col,[](int value){ return value > 30;});
for_each(filteredCol,lambda_echo);