"use strict";

import React from 'react';

import {SearchList} from '../components/SearchList';

import ItemService from '../services/ItemService';


/*
 * assuming the API returns something like this:
 *   const json = [
 *      { value: 'one', label: 'One' },
 *      { value: 'two', label: 'Two' }
 *   ]
 */

export class SearchView extends React.Component {

    constructor(props) {
        super(props);
        console.log("euaaaa",props);
        this.state = {
            loading: true,
            items: [],
            string: props.match.params.str
        };
    }
    componentWillMount(props){
        ItemService.searchItem(this.state.string).then((data) => {
            this.setState({loading: false, items: data})
        }).catch((e) => {
            console.error(e);
            this.setState(Object.assign({}, this.state, {error: 'Error while creating item'}));
        });

    }

    render() {
        if(this.state.loading){
            return(<h2>Loading...</h2>);
        }

        console.log("view",this.state.items);
        return(
                <SearchList  keywords={this.state.string} data = {this.state.items} user_evals={this.state.user_evals}/>

        );
    }
}
