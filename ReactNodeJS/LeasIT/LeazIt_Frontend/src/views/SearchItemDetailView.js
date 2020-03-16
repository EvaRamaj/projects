"use strict";

import React from 'react';

import SearchItemDetail from '../components/SearchItemDetail';

import ItemService from '../services/ItemService';


export class SearchItemDetailView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            loading: true,
            item: []
        }
    }

    componentWillMount(props){
        this.setState({
            loading: true
        });
        let id = this.props.match.params.id;
        console.log(id);
        ItemService.getItem(id).then((data) => {
            this.setState({
                item: data,
                loading: false
            });
        }).catch((e) => {
            console.error(e);
        });

    }

    deleteItem(id) {
        ItemService.deleteItem(id).then((message) => {
            this.props.history.push('/');
        }).catch((e) => {
            console.log(e);
        });
    }

    render() {
        if (this.state.loading) {
            return (<h2>Loading...</h2>);
        }

        return (
            <SearchItemDetail item={this.state.item} onDelete={(id) => this.deleteItem(id)}/>
        );
    }
}
